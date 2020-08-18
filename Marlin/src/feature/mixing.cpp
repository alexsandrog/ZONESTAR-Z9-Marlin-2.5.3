/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(MIXING_EXTRUDER)

//#define MIXER_NORMALIZER_DEBUG

#include "mixing.h"
#include "../module/motion.h"
#include "../module/planner.h"

Mixer mixer;

#ifdef MIXER_NORMALIZER_DEBUG
  #include "../core/serial.h"
#endif

// Used up to Planner level
uint_fast8_t  Mixer::selected_vtool = 0;

float         Mixer::collector[MIXING_STEPPERS]; // mix proportion. 0.0 = off, otherwise <= COLOR_A_MASK.

mixer_comp_t  Mixer::color[NR_MIXING_VIRTUAL_TOOLS][MIXING_STEPPERS];

// Used in Stepper
int_fast8_t   Mixer::runner = 0;
mixer_comp_t  Mixer::s_color[MIXING_STEPPERS];
mixer_accu_t  Mixer::accu[MIXING_STEPPERS] = { 0 };

mixer_perc_t Mixer::mix[MIXING_STEPPERS];


void Mixer::normalize(const uint8_t tool_index) {
  float cmax = 0;
#ifdef MIXER_NORMALIZER_DEBUG
  float csum = 0;
#endif

  MIXER_STEPPER_LOOP(i) {
	const float v = collector[i];  
    NOLESS(cmax, v);
    #ifdef MIXER_NORMALIZER_DEBUG
      csum += v;
    #endif
  }  

  #ifdef MIXER_NORMALIZER_DEBUG
    SERIAL_ECHOLNPGM("normalize");
	SERIAL_EOF();
    SERIAL_ECHOLNPAIR("tool_index=",uint16_t(tool_index));
    SERIAL_ECHOPGM("Mixer: Old relation : [ ");
    MIXER_STEPPER_LOOP(i) {
      SERIAL_ECHO_F(collector[i] / csum, 3);
      SERIAL_ECHOPGM(", ");
    }
    SERIAL_ECHOLNPGM("]");
	SERIAL_EOF();
  #endif

  // Scale all values so their maximum is COLOR_A_MASK
  const float scale = float(COLOR_A_MASK) / cmax;
  MIXER_STEPPER_LOOP(i) color[tool_index][i] = collector[i] * scale;

#if ENABLED(USE_PRECENT_MIXDATA)
  update_mix_from_vtool(tool_index);
#endif

  #ifdef MIXER_NORMALIZER_DEBUG
    csum = 0;
    SERIAL_ECHOPGM("Mixer: Normalize to : [ ");
    MIXER_STEPPER_LOOP(i) {
      SERIAL_ECHO(uint16_t(color[tool_index][i]));
      SERIAL_ECHOPGM(", ");
      csum += color[tool_index][i];
    }
    SERIAL_ECHOLNPGM("]");
    SERIAL_ECHOPGM("Mixer: New relation : [ ");
    MIXER_STEPPER_LOOP(i) {
      SERIAL_ECHO_F(uint16_t(color[tool_index][i]) / csum, 3);
      SERIAL_ECHOPGM(", ");
    }
    SERIAL_ECHOLNPGM("]");
	SERIAL_EOF();

    SERIAL_ECHOPGM("Mix Percent: [ ");
    MIXER_STEPPER_LOOP(i) {
      SERIAL_ECHO(uint16_t(mix[i]));
      SERIAL_ECHOPGM(", ");
    }
    SERIAL_ECHOLNPGM("]");
	SERIAL_EOF();
  #endif

  #if ENABLED(GRADIENT_MIX)
    refresh_gradient();
  #endif
}

void Mixer::reset_vtools() {
  // Virtual Tools 0, 1, 2, 3 = Filament 1, 2, 3, 4, etc.
  // Every virtual tool gets a pure filament
  LOOP_L_N(t, MIXING_VIRTUAL_TOOLS && t < MIXING_STEPPERS)
    MIXER_STEPPER_LOOP(i)
      color[t][i] = (t == i) ? COLOR_A_MASK : 0;

  // Remaining virtual tools are 100% filament 1
  #if MIXING_VIRTUAL_TOOLS > MIXING_STEPPERS
    LOOP_S_L_N(t, MIXING_STEPPERS, MIXING_VIRTUAL_TOOLS)
      MIXER_STEPPER_LOOP(i)
        color[t][i] = (i == 0) ? COLOR_A_MASK : 0;
  #endif

  #ifdef MIXER_NORMALIZER_DEBUG
  SERIAL_EOL();
  SERIAL_ECHOLNPGM("reset_vtools!");
  SERIAL_EOF();
  for(uint8_t t=0; t<MIXING_VIRTUAL_TOOLS; t++){
    for(uint8_t i=0; i<MIXING_STEPPERS; i++){ 
    	SERIAL_ECHOPGM("color[ ");
        SERIAL_ECHO(uint16_t(t));
		SERIAL_ECHOPGM("]");
		SERIAL_ECHOPGM("[");
		SERIAL_ECHO(uint16_t(i));
		SERIAL_ECHOPGM("]=");
		SERIAL_ECHO(uint16_t(color[t][i]));
		SERIAL_ECHOPGM(", ");
		SERIAL_EOF();
		SERIAL_EOL();
  	}
  }  
  #endif
}

// called at boot
void Mixer::init() {

  mixer.selected_vtool = 0;
  reset_vtools();

  #if ENABLED(RETRACT_SYNC_MIXING)
    // AUTORETRACT_TOOL gets the same amount of all filaments
    MIXER_STEPPER_LOOP(i)
      color[MIXER_AUTORETRACT_TOOL][i] = COLOR_A_MASK;
  #endif

  ZERO(collector);
  
  update_mix_from_vtool();
  mixer.copy_mix_to_collector();

  #if ENABLED(GRADIENT_MIX)
    update_gradient_for_planner_z();
  #endif

  #if ENABLED(RANDOM_MIX)
    //update_randommix_for_planner_z();
  #endif
}

void Mixer::refresh_collector(const float proportion/*=1.0*/, const uint8_t t/*=selected_vtool*/, float (&c)[MIXING_STEPPERS]/*=collector*/)
{
  float csum = 0, cmax = 0;
  MIXER_STEPPER_LOOP(i) {
	const float v = color[t][i];
	cmax = _MAX(cmax, v);
	csum += v;
  }

  #ifdef MIXER_NORMALIZER_DEBUG
  SERIAL_ECHOPAIR("Mixer::refresh_collector(", proportion, ", ", int(t), ") cmax=", cmax, "  csum=", csum, "	color");
  #endif
  
  const float inv_prop = proportion / csum;
  MIXER_STEPPER_LOOP(i) {
	c[i] = color[t][i] * inv_prop;	
	#ifdef MIXER_NORMALIZER_DEBUG
	SERIAL_ECHOPAIR(" [", int(t), "][", int(i), "] = ", int(color[t][i]), " (", c[i], ")	");
	#endif
  }
  #ifdef MIXER_NORMALIZER_DEBUG
  SERIAL_EOL();
  SERIAL_EOF();
  #endif
}


void Mixer::copy_mix_to_collector() {
  MIXER_STEPPER_LOOP(i){
  	collector[i] = (float)mix[i]/10;
  }
  #ifdef MIXER_NORMALIZER_DEBUG
    SERIAL_ECHOPGM("copy_mix_to_collector");
    SERIAL_EOF();
    SERIAL_ECHOPGM("Mix [ ");
    MIXER_STEPPER_LOOP(i){ 
		SERIAL_ECHO(int(mix[i]));
		SERIAL_ECHOPGM(", ");
    }
	SERIAL_ECHOPGM(" ] to collector [ ");
	MIXER_STEPPER_LOOP(i){ 
		SERIAL_ECHO(int(collector[i]));
		SERIAL_ECHOPGM(", ");
    }    
    SERIAL_ECHOLNPGM(" ]");
	SERIAL_EOF();
  #endif
}

void Mixer::copy_collector_to_mix() {
  MIXER_STEPPER_LOOP(i)	mix[i] = (mixer_perc_t)(collector[i]*10);
  int16_t sum_mix = 0;
  for(uint8_t i=0; i<MIXING_STEPPERS-1; i++) sum_mix += mix[i];
	  	mix[MIXING_STEPPERS-1] = 100 - sum_mix;
  
  #ifdef MIXER_NORMALIZER_DEBUG
    SERIAL_ECHOLNPGM("copy_collector_to_mix");
    SERIAL_EOF();
    SERIAL_ECHOPGM("collector [ ");
    MIXER_STEPPER_LOOP(i){ 
		SERIAL_ECHO(int(collector[i]));
		SERIAL_ECHOPGM(", ");
    }
	SERIAL_ECHOPGM(" ] to Mix [ ");
	MIXER_STEPPER_LOOP(i){ 
		SERIAL_ECHO(int(mix[i]));
		SERIAL_ECHOPGM(", ");
    }    
    SERIAL_ECHOLNPGM(" ]");
	SERIAL_EOF();
  #endif
}  


#if ENABLED(GRADIENT_MIX)

gradient_t Mixer::gradient = {
    false,    // enabled
    {0},      // color (array)
    0, 0,     // start_z, end_z
    0, 1,     // start_vtool, end_vtool
    {0}, {0}  // start_mix[], end_mix[]
    #if ENABLED(GRADIENT_VTOOL)
      , -1    // vtool_index
    #endif
};

float Mixer::prev_z; // = 0

void Mixer::update_gradient_for_z(const float z) {
    if (z == prev_z) return;
    prev_z = z;

    const float slice = gradient.end_z - gradient.start_z;

    float pct = (z - gradient.start_z) / slice;
    NOLESS(pct, 0.0f); NOMORE(pct, 1.0f);

    MIXER_STEPPER_LOOP(i) {
      const mixer_perc_t sm = gradient.start_mix[i];
      mix[i] = sm + (gradient.end_mix[i] - sm) * pct;
    }
    copy_mix_to_color(gradient.color);
}

void Mixer::update_gradient_for_planner_z() {
    update_gradient_for_z(planner.get_axis_position_mm(Z_AXIS));
}

#endif // GRADIENT_MIX

#if ENABLED(RANDOM_MIX)
randommix_t Mixer::random_mix = {
    false,    // enabled
    0, 0,     // start_z, end_z
    {0}
};

void Mixer::update_randommix_for_z(const float z) {
    if (z == prev_z) return;
	prev_z = z;	
	if(z >= random_mix.start_z && z <= random_mix.end_z){
    	MIXER_STEPPER_LOOP(i) mix[i] = random(100); 	
    	copy_mix_to_color(random_mix.color);
    }
}

void Mixer::update_randommix_for_planner_z() {
    update_randommix_for_z(planner.get_axis_position_mm(Z_AXIS));
}
#endif


#endif // MIXING_EXTRUDER

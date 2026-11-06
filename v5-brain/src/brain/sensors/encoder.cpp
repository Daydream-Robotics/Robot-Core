/** @file encoder.cpp
 *  @brief Implementation placeholder for the PROS rotation-sensor adapter.
 */
#include "./brain/sensors/encoder.hpp"

#include "pros/rotation.hpp"

#include <cstdint>

namespace brain {

   explicit Encoder::Encoder(std::int8_t smartPort, bool reversed){

   }

   void Encoder::update(){

   }

   double Encoder::position() const{

   }

   double Encoder::delta() const{

   }

   bool Encoder::connected() const{

   }

   void Encoder::tare(){
    
   }

}
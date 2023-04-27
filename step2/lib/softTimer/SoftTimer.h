#ifndef SoftTimerLib
#define SoftTimerLib
#include "Arduino.h"
class SoftTimer{
  private:
    unsigned long END_OF_TIMER = 0;
  public:
    // Run inside void loop
    inline void update(){
        if (is_time()&& (! is_stopped())){reset(); }
    }
    // Use it to Check if the timer is done 
    inline bool is_stopped(){ return (END_OF_TIMER == 0);}
    inline bool is_time(){ 
      if (is_stopped()) return 0;
      if ((END_OF_TIMER <= micros())) return 1;
      return 0;
    }
    inline void reset(){END_OF_TIMER=0;}
    inline void start(unsigned long millis_dt){
      if (is_stopped()){
        END_OF_TIMER = micros()+millis_dt*1000;
      }
    }
    inline void start_micros(unsigned long micros_dt){
      if (is_stopped()){
        END_OF_TIMER = micros()+micros_dt;
      }
    }
};
#endif
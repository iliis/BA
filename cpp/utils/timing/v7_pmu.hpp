/*
 * v7.h
 *
 *  Created on: Apr 16, 2014
 *      Author: Pascal Gohl
 */

#ifndef V7_H_
#define V7_H_

// ------------------------------------------------------------
// PMU for Cortex-A/R (v7-A/R)
// ------------------------------------------------------------

// Returns the number of progammable counters
unsigned int getPMN(void) __asm__("getPMN");

// Sets the event for a programmable counter to record
// counter = r0 = Which counter to program  (e.g. 0 for PMN0, 1 for PMN1)
// event   = r1 = The event code (from appropiate TRM or ARM Architecture Reference Manual)
void pmn_config(unsigned int counter, unsigned int event) __asm__("pmn_config");

// Enables/disables the divider (1/64) on CCNT
// divider = r0 = If 0 disable divider, else enable dvider
void ccnt_divider(int divider) __asm__("ccnt_divider");

//
// Enables and disables
//

// Global PMU enable
// On ARM11 this enables the PMU, and the counters start immediately
// On Cortex this enables the PMU, there are individual enables for the counters
void enable_pmu(void) __asm__("enable_pmu");

// Global PMU disable
// On Cortex, this overrides the enable state of the individual counters
void disable_pmu(void) __asm__("disable_pmu");

// Enable the CCNT
void enable_ccnt(void) __asm__("enable_ccnt");

// Disable the CCNT
void disable_ccnt(void) __asm__("disable_ccnt");

// Enable PMN{n}
// counter = The counter to enable (e.g. 0 for PMN0, 1 for PMN1)
void enable_pmn(unsigned int counter) __asm__("enable_pmn");

// Enable PMN{n}
// counter = The counter to enable (e.g. 0 for PMN0, 1 for PMN1)
void disable_pmn(unsigned int counter) __asm__("disable_pmn");

//
// Read counter values
//

// Returns the value of CCNT
static inline unsigned int read_ccnt(void) {
  uint32_t r = 0;
  asm volatile("mrc p15, 0, %0, c9, c13, 0" : "=r"(r) );
  return r;
}

// Returns the value of PMN{n}
// counter = The counter to read (e.g. 0 for PMN0, 1 for PMN1)
unsigned int read_pmn(unsigned int counter) __asm__("read_pmn");

//
// Overflow and interrupts
//

// Returns the value of the overflow flags
unsigned int read_flags(void) __asm__("read_flags");

// Writes the overflow flags
void write_flags(unsigned int flags) __asm__("write_flags");

// Enables interrupt generation on overflow of the CCNT
void enable_ccnt_irq(void) __asm__("enable_ccnt_irq");

// Disables interrupt generation on overflow of the CCNT
void disable_ccnt_irq(void) __asm__("disable_ccnt_irq");

// Enables interrupt generation on overflow of PMN{x}
// counter = The counter to enable the interrupt for (e.g. 0 for PMN0, 1 for PMN1)
void enable_pmn_irq(unsigned int counter) __asm__("enable_pmn_irq");

// Disables interrupt generation on overflow of PMN{x}
// counter = r0 =  The counter to disable the interrupt for (e.g. 0 for PMN0, 1 for PMN1)
void disable_pmn_irq(unsigned int counter) __asm__("disable_pmn_irq");

//
// Counter reset functions
//

// Resets the programmable counters
void reset_pmn(void) __asm__("reset_pmn");

// Resets the CCNT
void reset_ccnt(void) __asm__("reset_ccnt");

//
// Software Increment

// Writes to software increment register
// counter = The counter to increment (e.g. 0 for PMN0, 1 for PMN1)
void pmu_software_increment(unsigned int counter) __asm__("pmu_software_increment");

//
// User mode access
//
static inline void pmu_start(unsigned int counter_nr, unsigned int event){
  enable_pmu();              // Enable the PMU
//  reset_pmn();               // Reset the configurable counters
  pmn_config(counter_nr, event);       // Configure counter 0 to count event code 0x03
//  enable_pmn(counter_nr);             // Enable counter
}

static inline void pmu_start(unsigned int event0,unsigned int event1,unsigned int event2,unsigned int event3,unsigned int event4,unsigned int event5){

  enable_pmu();              // Enable the PMU
  reset_ccnt();              // Reset the CCNT (cycle counter)
  reset_pmn();               // Reset the configurable counters
  pmn_config(0, event0);       // Configure counter 0 to count event code 0x03
  pmn_config(1, event1);       // Configure counter 1 to count event code 0x03
  pmn_config(2, event2);       // Configure counter 2 to count event code 0x03
  pmn_config(3, event3);       // Configure counter 3 to count event code 0x03
  pmn_config(4, event4);       // Configure counter 4 to count event code 0x03
  pmn_config(5, event5);       // Configure counter 5 to count event code 0x03

  enable_ccnt();             // Enable CCNT
  enable_pmn(0);             // Enable counter
  enable_pmn(1);             // Enable counter
  enable_pmn(2);             // Enable counter
  enable_pmn(3);             // Enable counter
  enable_pmn(4);             // Enable counter
  enable_pmn(5);             // Enable counter

  printf("CountEvent0=0x%x,CountEvent1=0x%x,CountEvent2=0x%x,CountEvent3=0x%x,CountEvent4=0x%x,CountEvent5=0x%x\n", event0,event1,event2,event3,event4,event5);
}


static inline unsigned int pmu_stop(unsigned int counter_nr){

  disable_pmn(counter_nr);            // Stop counter
  unsigned int counter = read_pmn(counter_nr); // Read counter
  //overflow=read_flags();        //Check for overflow flag
  return counter;
}

static inline void pmu_stop(){
  unsigned int cycle_count, overflow, counter0, counter1, counter2, counter3, counter4, counter5;

  disable_ccnt();            // Stop CCNT
  disable_pmn(0);            // Stop counter 0
  disable_pmn(1);            // Stop counter 1
  disable_pmn(2);            // Stop counter 2
  disable_pmn(3);            // Stop counter 3
  disable_pmn(4);            // Stop counter 4
  disable_pmn(5);            // Stop counter 5

  counter0    = read_pmn(0); // Read counter 0
  counter1    = read_pmn(1); // Read counter 1
  counter2    = read_pmn(2); // Read counter 2
  counter3    = read_pmn(3); // Read counter 3
  counter4    = read_pmn(4); // Read counter 4
  counter5    = read_pmn(5); // Read counter 5

  cycle_count = read_ccnt(); // Read CCNT
  overflow=read_flags();        //Check for overflow flag

  printf("Counter0=%d,Counter1=%d,Counter2=%d,Counter3=%d,Counter4=%d,Counter5=%d\n", counter0, counter1,counter2,counter3,counter4,counter5);
  printf("Overflow flag: = %d, Cycle Count: = %d \n\n", overflow,cycle_count);
}

// Enables User mode access to the PMU (must be called in a priviledged mode)
void enable_pmu_user_access(void) __asm__("enable_pmu_user_access");

// Disables User mode access to the PMU (must be called in a priviledged mode)
void disable_pmu_user_access(void) __asm__("disable_pmu_user_access");

#endif /* V7_H_ */

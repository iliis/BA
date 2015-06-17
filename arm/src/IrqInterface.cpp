/*
 * IrqInterface.cpp
 *
 *  Created on: Nov 27, 2013
 *      Author: skybotix
 */

#include "IrqInterface.hpp"

#include <signal.h>
#include <iostream>
#include <stdio.h>
#include <string.h>

//lib
#include <unistd.h>
#include <fstream>
#include <stdlib.h>
#include <signal.h>
#include <map>
#include <deque>
#include <sys/time.h>
#include <boost/chrono.hpp>
#include <boost/thread/mutex.hpp>
#include "helpers.hpp"

//Example usage:
//=============
//	void MyCallback(int irqNr)
//	{
//		std::cout << "Received IRQ: " << irqNr << "\n";
//		return;
//	}
//
//	int main()
//	{
//	  /* register IRQ callback */
//	  IrqInterface irq( MyCallback );
//
//	  while(1)
//	  {
//		irq.printStatistics();
//		sleep(1);
//	  }
//	  return 0;
//	}
//
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !! Use max. one instance or the statistics will be messed up !!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



//signal handler wrapper
IrqInterface *handlerClass=NULL;
void sharedSignalISR_wrapper(int sigNum, siginfo_t *si, void *ucontext)
{
	handlerClass->sharedSignalISR(sigNum, si, ucontext);
}


//irq statistics
struct IrqInterface::IrqStatistic {
	const int IRQ;	  //irq number the statistic is about
	long count;       //count

	IrqStatistic(int IRQ) :
		IRQ(IRQ),
		N_HIST(20)
	{
		count = 0;
		boost::chrono::high_resolution_clock::time_point timestamp = boost::chrono::high_resolution_clock::now();

		for (int i = 0; i < N_HIST; i++)
			timestamp_history.push_back(timestamp);
	}

	void addTimestamp(boost::chrono::high_resolution_clock::time_point timestamp) {
		timestamp_history.push_back(timestamp);
		timestamp_history.pop_front();
	}

	// get frequency in [Hz]
	double getFrequency() {
		boost::chrono::nanoseconds deltaTns;

		//sum all time differences in sample window
		for (int i = 0; i < N_HIST - 1; i++)
			deltaTns += (timestamp_history[i+1] - timestamp_history[i]);

		//average equally over window and convert to freq [Hz]
		double deltaT = 1.0/(double)N_HIST * deltaTns.count()/(double)1e9;

		//return freq
		return 1.0/deltaT;
	}

#if 0
	void printWindow() {
		std::cout << "\n";
		for (int i = 0; i < N_HIST - 1; i++)
			std::cout << timestamp_history[i] << "\n";

		std::cout << "....\n";
	}
#endif

private:
	const int N_HIST;     //history window

	//history of timestamps for frequency calculations (fifo)
	std::deque< boost::chrono::high_resolution_clock::time_point > timestamp_history;

};

IrqInterface::IrqInterface(IrqScheduler *sched) {
	//set the callback
	pIrq_sched = sched;

	//register class
	if(handlerClass != NULL)
	{
		std::cout << "You can have only one instance of the IrqInterface!!\n";
		exit(-1);
	}

	//store class pointer for handler wrapper
	handlerClass = (IrqInterface*)this;

	//connect to kernel driver
	attachKernelSignal();
	registerIRQdriver();
}

IrqInterface::~IrqInterface() {
	//disconnect from kernel driver
	unregisterIRQdriver();
}

void IrqInterface::sharedSignalISR(int sigNum, siginfo_t *si, void *ucontext) {
	//check signal number
	if (sigNum != SIGIO)
		return;

	//receive the interrupt number
	int IRQ = si->_sifields._rt.si_sigval.sival_int;

	// call external callback
	pIrq_sched->irqCallback(IRQ);

	//update statistics
#ifdef DO_STATISTICS
	addStatistics(IRQ);
#endif
}

void IrqInterface::attachKernelSignal() {
	//define signal action
	struct sigaction action;
	memset(&action, 0, sizeof(action));
	action.sa_sigaction = sharedSignalISR_wrapper;
	action.sa_flags = SA_SIGINFO;  //payload

	//attach callback to IO signal
	sigaction(SIGIO, &action, NULL);
}

void IrqInterface::registerIRQdriver() {
	//register our pid with the irq driver
	pid_t myPid = getpid();

	FILE * irqDriver;
	irqDriver = fopen("/proc/visensor-isr", "w+");
	if (irqDriver != NULL) {
		char myPidStr[6];
		sprintf(myPidStr, "%d", myPid);
		fputs(myPidStr, irqDriver);
		fclose(irqDriver);

	} else {
		std::cout << "ERROR: visensor-isr module not loaded!\n";
		exit(EXIT_FAILURE);
	}
}

void IrqInterface::unregisterIRQdriver() {
	FILE * irqDriver;
	irqDriver = fopen("/proc/visensor-isr", "w+");

	if (irqDriver != NULL) {
		fputs("0", irqDriver); //disconnect with registering PID:0
		fclose(irqDriver);
	}
}

////////////////////////////////////////////////////
// IRQ STATISTICS
////////////////////////////////////////////////////

void IrqInterface::printStatistics() {
	boost::unique_lock<boost::mutex> lock(m_statistics_);

	AtomicWriter w;

	//header
	w << "IRQ: \t count: \t freq: \n";

	//check if we have any data
	if (irqStatistics.size() < 1) {
		w << "No data.\n";
		return;
	}

	//data rows
	for (std::map<int, IrqStatistic>::iterator iterator = irqStatistics.begin();
			iterator != irqStatistics.end(); iterator++) {
		w << iterator->second.IRQ << "\t" << iterator->second.count
				<< "\t\t" << iterator->second.getFrequency() << "\n";

	}
}

//TODO(schneith): make thread safe...
inline void IrqInterface::addStatistics(int IRQ) {
	boost::unique_lock<boost::mutex> lock(m_statistics_);

	//get a timestamp for the interrupt
    boost::chrono::high_resolution_clock::time_point timestamp = boost::chrono::high_resolution_clock::now();

	//check if the given IRQ is already tracked
	//else create an entry...
	if (irqStatistics.count(IRQ) == 0) {
		IrqStatistic empty_stat(IRQ);
		irqStatistics.insert( { IRQ, empty_stat });
	}

	//update the counter
	irqStatistics.find(IRQ)->second.count++;

	//add timestamp for frequency calculation
	irqStatistics.find(IRQ)->second.addTimestamp(timestamp);

}


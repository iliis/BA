/*
 * IrqInterface.hpp
 *
 *  Created on: Nov 27, 2013
 *      Author: skybotix
 */

#ifndef IRQINTERFACE_HPP_
#define IRQINTERFACE_HPP_

#include <map>
#include <signal.h>
#include <deque>

#include <boost/thread/mutex.hpp>

#include "IrqScheduler.hpp"

//switch to turn on statistics
//(consumes some time for each interrupt)
#define DO_STATISTICS

class IrqScheduler;

class IrqInterface {
public:

	IrqInterface(IrqScheduler *sched);

	~IrqInterface();
	void printStatistics();

	void sharedSignalISR(int sigNum, siginfo_t *si, void *ucontext);

private:

	void attachKernelSignal();
	void registerIRQdriver();
	void unregisterIRQdriver();

	inline void addStatistics(int IRQ);

	//irq statistics
	struct IrqStatistic;

	//statistics container
	boost::mutex m_statistics_;
	std::map<int, IrqStatistic> irqStatistics;

	//external irq scheduler with callbacks init
	IrqScheduler *pIrq_sched;

};

#endif /* IRQINTERFACE_HPP_ */

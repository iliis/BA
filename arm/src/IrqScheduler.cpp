
#include "IrqScheduler.hpp"

#include <queue>

#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include "IrqInterface.hpp"
#include "helpers.hpp"

IrqScheduler::IrqScheduler(std::vector<Sensor*> sensor) :
	irq_( new IrqInterface( this ) ), //TODO(schneith): does this leak memory?
	active_(false),					  // set inactive (dont stream data)
	sensor_(sensor)
{
  //priority chain (top element = highest priority)
  irqQueue_[ irqQueues::HIGH_PRIORITY ].parent_queue = NULL;
  irqQueue_[ irqQueues::MED_PRIORITY ].parent_queue = &irqQueue_[ irqQueues::HIGH_PRIORITY ];
  irqQueue_[ irqQueues::LOW_PRIORITY ].parent_queue = &irqQueue_[ irqQueues::MED_PRIORITY ];

  // start processing thread
  boost::thread consumer( boost::bind(&IrqScheduler::processIrqQueues, this) );

}

IrqScheduler::~IrqScheduler()
{
	delete irq_;
}

void IrqScheduler::setActive(bool onoff)
{
	active_ = onoff;
}

//no work in this function!
void IrqScheduler::irqCallback(int irqNr)
{
  //only add data to queues if the scheduler is active
  if(!active_)
    return;

  bool newData = true;

  /* add irq to the corresponding queue with desired priority */
  switch(irqNr)
  {
    case FPGA_CONSTANTS::IRQ::IMU0: //imu0
      irqQueue_[ irqQueues::HIGH_PRIORITY ].push( irqNr );
      break;

    /* stereo camera */
    case FPGA_CONSTANTS::IRQ::CAM0: //cam0
    case FPGA_CONSTANTS::IRQ::CAM1: //cam1
      irqQueue_[ irqQueues::MED_PRIORITY ].push( irqNr );
      break;

    /* external camera modules */
    case FPGA_CONSTANTS::IRQ::CAM2: //cam2
    case FPGA_CONSTANTS::IRQ::CAM3: //cam3
      irqQueue_[ irqQueues::LOW_PRIORITY ].push( irqNr );
      break;

    default:
      std::cout << "Received unhandled IRQ " << irqNr << "\n";
      exit(-1);
      break;
  }

  //notify the queue handler that one queue has new data
  if(newData)
    cv_irq_received_.notify_one();
}


// pass the measurement to the tcp server for sending
void IrqScheduler::streamMeasurement(size_t sensor_id)
{
	//send out measurement here
//    IpComm::Header header;
//    header.timestamp = 0;
//    header.data_size = sensor_[sensor_id]->data_size();
//    header.data_id = 100+sensor_id;
//    imu_server.send_data(sensor_[sensor_id]->data(), header_imu);
    sensor_[sensor_id]->data_mover()->movePointer();
}


//priority scheduling algorithm
void IrqScheduler::processIrqPrioritizedQueues(ConcurrentQueue<int> *irq_queue)
{
  // empty entire irq queue
  std::queue<int> queue = irq_queue->pop_all();

  //if the queue is empty just process the parent queue
  if( queue.empty() )
    if(irq_queue->parent_queue != NULL)
      processIrqPrioritizedQueues(irq_queue->parent_queue);

  // go through all elements
  while(!queue.empty())
  {
    //empty parent queue first (higher priority queue)
    if(irq_queue->parent_queue != NULL)
      processIrqPrioritizedQueues(irq_queue->parent_queue);

    //process one element
    int irq_nr = queue.front();
    size_t sensor_id = irqSensor[ irq_nr ]; //map irq_nr to sensor_id

    //send out the measurement
    streamMeasurement( sensor_id );

    //remove the processed queue element
    queue.pop();

  }
}


// empty all queues considering queue priorities
// this thread is woken up by irq events
void IrqScheduler::processIrqQueues()
{
  boost::unique_lock<boost::mutex> lock(m_processIrq_);

  while(1)
  {
    // wait for new data in the queues
    cv_irq_received_.wait(lock);

    dumpQueues();

    //process the queues
    processIrqPrioritizedQueues( &irqQueue_[ irqQueues::LOW_PRIORITY ] );
  }
}

//debug function
void IrqScheduler::dumpQueues()
{
  AtomicWriter w;
  w << "Current queue content:\n";
  w << "--------------------------\n";

  std::queue<int> queue;

  w << "High Priority: ";
  for (queue = irqQueue_[ irqQueues::HIGH_PRIORITY ].clone(); !queue.empty(); queue.pop())
       w << queue.front() << ", ";
  w << "\n";

  w << "Med Priority: ";
  for (queue = irqQueue_[ irqQueues::MED_PRIORITY ].clone(); !queue.empty(); queue.pop())
       w << queue.front() << ", ";
  w << "\n";

  w << "Low Priority: ";
  for (queue = irqQueue_[ irqQueues::LOW_PRIORITY ].clone(); !queue.empty(); queue.pop())
       w << queue.front() << ", ";
  w << "\n";
  w << "--------------------------\n";
}


void IrqScheduler::printIrqStatistics()
{
	irq_->printStatistics();
}

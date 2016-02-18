import rospy
from Queue import Queue, Empty as EmptyQueueException
from threading import Condition

# The global callback queue
callback_queue = Queue()

def spin():
    '''
    This spinner is used in place of the rospy.spin() method. Whereas
    rospy.spin() simply waits for rospy.is_shutdown(), this spinner processes
    the callback queue until rospy.is_shutdown() is true. The callback
    queue is thread-safe, so for a multi-threaded queue processor, call this
    spin method in multiple threads.
    '''
    while not rospy.is_shutdown():
        try:
            condition = callback_queue.get(block=True, timeout=0.1)
        except EmptyQueueException:
            pass
        else:
            with condition:
                condition.notify()
                condition.wait()

def single_threaded(callback):
    '''
    This decorator can be used on service or subscriber callbacks. A callback
    wrapped with this decorator will put a condition on the callback_queue and
    then wait its turn before executing the decorated callback.
    '''
    def wrapped_callback(arg):
        # Put condition on queue
        condition = Condition()
        with condition:
            callback_queue.put(condition)
            # Wait for condition
            condition.wait()
            # Do the callback
            ret = callback(arg)
            condition.notify()
            return ret
    return wrapped_callback

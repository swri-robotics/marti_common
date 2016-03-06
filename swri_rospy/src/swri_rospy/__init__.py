import rospy
from rospy.impl.tcpros_base import DEFAULT_BUFF_SIZE
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

# Override the default rospy spin() with swri_rospy.spin()
rospy.spin = spin

def single_threaded(callback):
    '''
    This decorator can be used on service or subscriber callbacks. A callback
    wrapped with this decorator will put a condition on the callback_queue and
    then wait its turn before executing the decorated callback.
    '''
    # Prevent recursive decoration, which would deadlock
    if hasattr(callback, 'single_threaded') and callback.single_threaded:
        return callback

    def wrapped_callback(*args, **kwds):
        # Put condition on queue
        condition = Condition()
        with condition:
            callback_queue.put(condition)
            # Wait for condition
            condition.wait()
            # Do the callback
            try:
                ret = callback(*args, **kwds)
            finally:
                condition.notify()
            return ret
    wrapped_callback.single_threaded = True
    return wrapped_callback

def service_wrapper(func):
    '''
    This is a decorator for functions that handle ROS service calls.
    It catches unhandled exceptions and reports them cleanly as errors
    on the ROS console.
    '''
    def wrapper(*args, **kwds):
        try:
            return func(*args, **kwds)
        except:
            rospy.logerr(traceback.format_exc())
            return None
    return wrapper

class Service(rospy.Service):
    def __init__(self, name, service_class, handler,
                 buff_size=DEFAULT_BUFF_SIZE, error_handler=None,
                 asynchronous=False):
        if not asynchronous:
            handler = single_threaded(handler)
        super(Service, self).__init__(name, service_class, handler, buff_size,
                                      error_handler)

class Subscriber(rospy.Subscriber):
    def __init__(self, name, data_class, callback=None, callback_args=None, 
                 queue_size=None, buff_size=DEFAULT_BUFF_SIZE,
                 tcp_nodelay=False, asynchronous=False):
        if not asynchronous:
            callback = single_threaded(callback)
        super(Subscriber, self).__init__(name, data_class, callback,
              callback_args, queue_size, buff_size, tcp_nodelay)

class Timer(rospy.Timer):
    def __init__(self, period, callback, oneshot=False, asynchronous=False):
        if not asynchronous:
            callback = single_threaded(callback)
        super(Timer, self).__init__(period, callback, oneshot)

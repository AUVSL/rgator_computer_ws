import can
from can.message import Message
 
# class MyListener(can.Listener):
#     def __init__(self):
#         super(MyListener, self).__init__()
 
#     def on_message_received(self, msg: Message) -> None:
#         """
#         example
#         when receive message 0x123，transmit message 0x456
#         """
#         if msg.arbitration_id == 0x0CF00400:
#             print("CAN msg received")
	
 
 
def print_msg(msg):
    if msg.arbitration_id == 0x0CF00400:
            print("CAN msg received")
            print(msg)
 
 
if __name__ == "__main__":
    # bus = can.interface.Bus('virtual_ch', bustype='virtual')
    # bus = can.interface.Bus(channel='can0', bustype='socketcan')
    bus = can.Bus(interface='socketcan', channel='can0', bitrate=250000)
    # logger = can.Logger("logfile.asc")  # save log to asc file
    listeners = [
        print_msg,  # Callback function, print the received messages
        # logger,  # save received messages to asc file
        # MyListener   # my listener
    ]
    notifier = can.Notifier(bus, listeners)
    running = True
    while running:
        input()
        running = False
 
    # It's important to stop the notifier in order to finish the writting of asc file
    notifier.stop()
    # stops the bus
    bus.shutdown()

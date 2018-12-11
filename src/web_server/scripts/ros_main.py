#!/usr/bin/emv python

import rospy
from server import *

def __log(status="INFO", message=""):
    rospy.loginfo("[{status}] {message}".format(status=status, message=message))

def main():
    __log("initializing node {}".format(NODE_NAME))
    
    rospy.init_node("{}".format(NODE_NAME), anonymous=False)
    rospy.on_shutdown(shutdown_server)

    __log("Starting web server")
    thread.start_new_thread(app.run, (HOST, PORT))

    time.sleep(5)
    __log("Server should be running")

    rospy.spin()

if __name__ == "__main__":
    main()


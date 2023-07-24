#!/usr/bin/env python3
#! -*- coding: utf-8 -*-

import rospy
from bumperbot_examples.srv import AddTwoInts, AddTwoIntsResponse
import sys

if __name__ == "__main__":
    # sys kütüphanesinin niye kullanıldığını yazmayı unutma
    if len(sys.argv) == 3:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
        print("Requesing ", a, b)
        rospy.wait_for_service("add_two_ints")
        add_two_ints = rospy.ServiceProxy("add_two_ints", AddTwoInts)
        response = add_two_ints(a, b)
        print("Service response", response)

    else:
        print("Requested two arguments")
        sys.exit(-1)
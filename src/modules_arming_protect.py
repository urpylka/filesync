    accept_operation = Event()
    accept_operation.set()

    def arming_protect(data):
        if data.armed and ARMING_PROTECT:
            if accept_operation.is_set():
                accept_operation.clear()
                print("ARMING все операции кроме upload заблокированы")
        else:
            if not accept_operation.is_set():
                accept_operation.set()
                print("DISARMING все операции разблокированы")


    rospy.Subscriber('/mavros/state', State, arming_protect)
#!/usr/bin/env/ python
# This Class is used to run a Thread
from threading import Thread
from multiprocessing import Process


class NewThread(Thread):
    def __init__(self, threadID, name, objType, objMeth, objArg):
        Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.objType = objType
        self.objArg = objArg
        self.objMeth = objMeth

    def run(self):
        threadToRun = self.objType
        getattr(threadToRun, self.objMeth)(self.objArg)

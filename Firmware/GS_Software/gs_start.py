from multiprocessing import Process
from livecam_gui import main_lc
from telemetry_gui import main_tel

if __name__ == '__main__':
    p1 = Process(target=main_lc)
    p2 = Process(target=main_tel)
    p1.start()
    p2.start()
    p1.join()
    p2.join()
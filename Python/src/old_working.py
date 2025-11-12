import time
import serial
import concurrent.futures
import multiprocessing as mp
import numpy as np
from PIL import ImageGrab
from PIL import Image

'''
Basert på youtube vid
youtube: https://www.youtube.com/watch?v=3dt4OGnU5sM
github: https://github.com/CoreyMSchafer/code_snippets/blob/master/Python/MultiProcessing/multiprocessing-demo.py
'''

resolution = [3440, 1440]
diviationAmt = 8
resizeResolution = [int(resolution[0] / diviationAmt), int(resolution[1] / diviationAmt)]
horizontalLED = 51
verticalLED = 22
LEDpersector = 3  # Only works with a value of 2 or 3 currently, dunno why
AmtLED = (horizontalLED * 2) + (verticalLED * 2)
Hsectors = int(horizontalLED / LEDpersector)
Vsectors = int(verticalLED / LEDpersector)
HOrigPix = int(resolution[0] / Hsectors)
VOrigPix = int(resolution[1] / Vsectors)
HResizePix = int(resolution[0] / Hsectors / diviationAmt)
VResizePix = int(resolution[1] / Vsectors / diviationAmt)
Sectors = np.zeros((Hsectors * 2 + Vsectors * 2 - 4))
ser = serial.Serial('COM3', 115200)
weights = [1.2, 0.9,
           0.8]  # Weights used for colors, not sure how they affect it beside making some of the be more impactful

TXStartPix = np.linspace(0, HOrigPix * (Hsectors - 1), Hsectors).astype(int)
TYStartPix = [0] * Hsectors
TStartPix = np.stack((TXStartPix, TYStartPix), axis=1)

BXStartPix = np.linspace(HOrigPix * (Hsectors - 1), 0, Hsectors).astype(int)
BYStartPix = [VOrigPix * (Vsectors - 1)] * Hsectors
BStartPix = np.stack((BXStartPix, BYStartPix), axis=1)

RYStartPix = np.linspace(VOrigPix, VOrigPix * (Vsectors - 2), Vsectors - 2).astype(int)
RXStartPix = [HOrigPix * (Hsectors - 1)] * (Vsectors - 2)
RStartPix = np.stack((RXStartPix, RYStartPix), axis=1)

LYStartPix = np.linspace(VOrigPix * (Vsectors - 2), VOrigPix, Vsectors - 2).astype(int)
LXStartPix = [0] * (Vsectors - 2)
LStartPix = np.stack((LXStartPix, LYStartPix), axis=1)

StartPix = np.concatenate(
    (TStartPix / diviationAmt, RStartPix / diviationAmt, BStartPix / diviationAmt, LStartPix / diviationAmt)).astype(
    int)  # Start value of all the sectors in pixels
EndPix = StartPix + [HOrigPix, VOrigPix]  # End value of all secors in pixels
XStart = np.zeros(len(StartPix)).astype(int)
YStart = np.zeros(len(StartPix)).astype(int)

for i in range(len(StartPix)):
    XStart[i] = StartPix[i][0]
    YStart[i] = StartPix[i][1]


def captureImage():
    # with mss.mss() as sct:
    #     image = sct.shot()
    image = ImageGrab.grab().resize([resizeResolution[0], resizeResolution[1]], ImageGrab.Image.Resampling.BOX)
    # print("Selfie acquired! :)")
    return image


def cropImage(x_start, y_start):
    cropImg = image.crop((x_start, y_start, x_start + HResizePix, y_start + VResizePix))
    sBox = cropImg.resize((1, 1), ImageGrab.Image.Resampling.BILINEAR)
    color = sBox.load()[0, 0]
    gamma_corrected_color = gammaCorrect(color[0], color[1],color[2])

    #print("Main color: ", color, "Gamma corrected color: ", "\n")

    #return color
    return gamma_corrected_color


def sendArduino():
    index = 0
    for i in range(len(Sectors)):
        for j in range(LEDpersector):
            send = []
            # time.sleep(0.005)
            for r in range(3):
                send.append(int(colors[i][r] * weights[r]))
                send[r] = int(min(send[r], 255))
            cmd = ("<" + str(index) + "," + str(send[0]) + "," + str(send[1]) + "," + str(send[2]) + ">")

            # print(cmd)
            ser.write(cmd.encode())
            index += 1

def gammaCorrect(r,g,b, gamma=2.2):
    """Apply gamma correction fo RGB values (0-255)"""
    def correct_channel(v):
        return max(0, min(255, int(((v / 255) ** gamma) * 255)))

    return tuple(correct_channel(v) for v in (r, g, b))

'--------------------------------------------------Main code here----------------------------------------------------------'

if __name__ == '__main__':
    mp.freeze_support()
    try:

        print("LEDS RUNNING")
        while True:
            start = time.perf_counter()
            image = captureImage()
            colors = []

            with concurrent.futures.ThreadPoolExecutor() as executor:
                croppedImg = executor.map(cropImage, XStart, YStart)
                for result in croppedImg:
                    colors.append(np.array(result).astype(int))
            sendArduino()

            finish = time.perf_counter()
            # print(f'Finished in {round(finish-start, 2)} second(s)')
    except Exception as e:
        print("LEDS STOPPED")
        print(e)

'--------------------------------------------------------------------------------------------------------------------------'

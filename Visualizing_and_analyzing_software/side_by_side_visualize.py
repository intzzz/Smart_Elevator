import numpy as np
from scipy import ndimage
import sys
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.widgets import Button
import matplotlib.animation as animation

## Global variables
Matrix_size = 32
N = 77
current_data = 0
image_array = []
thermo_array = []
thermo_array_temp = []
stopped = True
reverse = False
playing = False

## Init
fig, (ax1, ax2) = plt.subplots(ncols=2)

with open('data1/serial_log.txt', 'r') as f:
    conf_arr = [[float(num) for num in line.split(',')] for line in f]

def calc_avg_temp(array):
    avg = 0.0
    tmp = 0.0
    i = 0
    for x in range (1024):
        avg += array[x]
        if (array[x] > 25):
            i += 1
            tmp += array[x]
    avg = avg/1024
    print(avg)
    if(i > 0):
        tmp/=i
        print(tmp)
        print(i)

def rotateMatrix(mat):
    # Consider all squares one by one
    for x in range(0, int(Matrix_size / 2)):
        # Consider elements in group
        # of N in current square
        for y in range(x, Matrix_size - x - 1):
            # store current cell in temp variable
            temp = mat[x][y]
            # move values from right to top
            mat[x][y] = mat[y][Matrix_size - 1 - x]
            # move values from bottom to right
            mat[y][Matrix_size - 1 - x] = mat[Matrix_size - 1 - x][Matrix_size - 1 - y]
            # move values from left to bottom
            mat[Matrix_size - 1 - x][Matrix_size - 1 - y] = mat[Matrix_size - 1 - y][x]
            # assign temp to left
            mat[Matrix_size - 1 - y][x] = temp

for x in range(N):
    image_array.append(mpimg.imread('data1/' + str(x) + '_PICTURE.jpg'))
    thermo_array.append(np.reshape(conf_arr[x], (Matrix_size, Matrix_size)))

#print(thermo_array[0])
#print("-----------------------------")
for x in range(N):
    rotateMatrix(thermo_array[x])
    rotateMatrix(thermo_array[x])
    rotateMatrix(thermo_array[x])

#print(thermo_array[0])
ax1.set_title("Pi Camera")
ax2.set_title("Thermal Camera")
res = ax2.imshow(np.array(thermo_array[current_data]), cmap=plt.cm.jet,interpolation='nearest', vmin=20, vmax=35)
ax1.imshow(image_array[current_data])
plt.colorbar(res, fraction=0.046, pad=0.05)
ax1.axis("off")

## Display temperature numbers
def display_temp_nr(resized_matrix):
    for x in range(Matrix_size):
        for y in range(Matrix_size):
            ax2.annotate(str(resized_matrix[x][y]), xy=(y, x),
                        horizontalalignment='center',
                        verticalalignment='center')


#display_temp_nr(thermo_array[current_data])

## Axe numbers
numbers = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31]
plt.xticks(range(Matrix_size), numbers[:Matrix_size])
plt.yticks(range(Matrix_size), numbers[:Matrix_size])

## Not to overlap
plt.tight_layout()

## Button presses
def press(event):
    print('press', event.key)
    sys.stdout.flush()
    if event.key == 'right':
        Index.next(callback, event)

    if event.key == 'left':
        Index.prev(callback, event)

class Index(object):
    def next(self, event):
        if (stopped == True):
            global current_data
            current_data += 1
            if current_data == N:
                current_data = 0
            print("NEXT")
            ax1.clear()
            ax2.clear()
            ax1.set_title("Pi Camera")
            ax2.set_title("Thermal Camera")
            ax1.imshow(image_array[current_data])
            ax2.imshow(np.array(thermo_array[current_data]), cmap=plt.cm.jet, interpolation='nearest', vmin=20, vmax=35)
            #display_temp_nr(thermo_array[current_data])
            ax1.axis("off")
            fig.canvas.draw()
            calc_avg_temp(conf_arr[current_data])

    def prev(self, event):
        if (stopped == True):
            global current_data
            current_data -= 1
            if current_data == -1:
                current_data = N - 1
            print("PREV")
            ax1.clear()
            ax2.clear()
            ax1.set_title("Pi Camera")
            ax2.set_title("Thermal Camera")
            ax1.imshow(image_array[current_data])
            ax2.imshow(np.array(thermo_array[current_data]), cmap=plt.cm.jet, interpolation='nearest', vmin=20, vmax=35)
            #display_temp_nr(thermo_array[current_data])
            ax1.axis("off")
            fig.canvas.draw()
            calc_avg_temp(conf_arr[current_data])

    def reverse(self, event):
        print("Reverse")
        global playing, stopped, reverse
        playing = False
        stopped = False
        reverse = True

    def stop(self, event):
        print("Stopping")
        global playing, stopped, reverse
        playing = False
        stopped = True
        reverse = False

    def play(self, event):
        print("PLAYING")
        global playing, stopped, reverse
        playing = True
        stopped = False
        reverse = False


def animate(i):
    global current_data
    if (playing == True):
        print("forward")
        current_data += 1
        if current_data == N:
            current_data = 0
    if(reverse == True):
        print("Backward")
        current_data -= 1
        if current_data == -1:
            current_data = N - 1
    if(stopped == False):
        print("playing video")
        ax1.clear()
        ax2.clear()
        ax1.set_title("Pi Camera")
        ax2.set_title("Thermal Camera")
        ax1.imshow(image_array[current_data])
        ax2.imshow(np.array(thermo_array[current_data]), cmap=plt.cm.jet, interpolation='nearest', vmin=20, vmax=35)
        #display_temp_nr(thermo_array[current_data])
        ax1.axis("off")
        fig.canvas.draw()


## Buttons
callback = Index()

axreverse = plt.axes([0.1, 0.05, 0.1, 0.075])
axstop = plt.axes([0.21, 0.05, 0.1, 0.075])
axplay = plt.axes([0.32, 0.05, 0.1, 0.075])
axnext = plt.axes([0.81, 0.05, 0.1, 0.075])
axprev = plt.axes([0.7, 0.05, 0.1, 0.075])

breverse = Button(axreverse, 'Reverse')
bstop = Button(axstop, 'Stop')
bplay = Button(axplay, 'Play')
bnext = Button(axnext, 'Next')
bprev = Button(axprev, 'Previous')

breverse.on_clicked(callback.reverse)
bstop.on_clicked(callback.stop)
bplay.on_clicked(callback.play)
bnext.on_clicked(callback.next)
bprev.on_clicked(callback.prev)

fig.canvas.mpl_connect('key_press_event', press)

## Play video
ani = animation.FuncAnimation(fig, animate, interval=1000)
plt.show()
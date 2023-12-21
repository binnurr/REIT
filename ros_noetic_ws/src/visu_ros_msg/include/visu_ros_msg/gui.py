
import PIL.Image
from tkinter import *
import tkinter.font as tkFont
from PIL import ImageTk, Image
import cv2
import os
from rospkg import RosPack
import time


class GUI(Tk):
    #*args, **kwargs
    def __init__(self, subject_name, session):
        Tk.__init__(self)

        # the container is where we'll stack a bunch of frames
        # on top of each other, then the one we want visible
        # will be raised above the others
        self.title("Virtual Customer Screen")
        self.geometry('1000x480')
        container = Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}
        for F in (VideoGUI, ConvGUI):
            page_name = F.__name__
            frame = F(container, self, subject_name, session)
            self.frames[page_name] = frame

            # put all of the pages in the same location;
            # the one on the top of the stacking order
            # will be the one that is visible.
            frame.grid(row=0, column=0, sticky="nsew")

        self.run_gui(["VideoGUI"])

    def run_gui(self, page_name):
        """
        '''Show a frame for the given page name'''
        frame = self.frames[page_name]
        frame.tkraise()
        """

        '''Show a frame for the given page name'''
        for frame in self.frames.values():
            frame.grid_remove()
        for f in page_name:
            frame = self.frames[f]
            frame.grid(column=frame.col)


class ConvGUI(Frame):
    def __init__(self, parent, controller, subject_name, session):
        Frame.__init__(self, parent, bg="black")
        #self.window = Tk()
        #self.window.title("RE Interview")
        #self.window.geometry('650x700')
        #self.window.configure(bg='black')

        self.controller = controller
        self.side = RIGHT
        self.col = 1

        """
        self.prev_answer_lbl = Label(self, text=" ", bg="black", fg="#f00",
                                      pady = 30, font=tkFont.Font(family = "Arial", size=14, slant="italic"),
                                     wraplength=600, justify="left")
        self.prev_answer_lbl.grid(column=2, row=1, sticky=W)
        """
        #self.prev_answer_lbl.pack()

        self.const_notify_lbl = Label(self, text=" ", bg="black", fg="white",
                                     pady = 20, font=tkFont.Font(family = "Arial", size=14, weight="bold"),
                                     wraplength=600, justify="left")
        self.const_notify_lbl.grid(column=2, row=1, sticky=W)
        #self.const_notify_lbl.pack()

        self.ques1_lbl = Label(self, text=" ", bg="black", fg="#40E0D0",
                                     pady = 10, font=tkFont.Font(family = "Times", size=14, weight="bold"),
                                     wraplength=600, justify="left")
        self.ques1_lbl.grid(column=2, row=2, sticky=W)
        #self.ques1_lbl.pack()

        self.ques2_lbl = Label(self, text=" ", bg="black", fg="#40E0D0",
                                     pady = 10, font=tkFont.Font(family = "Times", size=14, weight="bold"),
                                     wraplength=600, justify="left")
        self.ques2_lbl.grid(column=2, row=3, sticky=W)
        #self.ques2_lbl.pack()

        self.ques3_lbl = Label(self, text=" ", bg="black", fg="#40E0D0",
                                      pady = 10, font=tkFont.Font(family = "Times", size=14, weight="bold"),
                                     wraplength=600, justify="left")
        self.ques3_lbl.grid(column=2, row=4, sticky=W)
        #self.ques3_lbl.pack()


        """
        text = Text(self.window, height=1, font="Helvetica 12")
        text.tag_configure("bold", font="Helvetica 12 bold")

        text.insert("end", "Hello, ")
        text.insert("end", "world", "bold")
        text.configure(state="disabled")
        """

        #txt = Entry(window,width=10)
        #txt.grid(column=1, row=0)

        #def clicked():
            #lbl.configure(text="Button was clicked !!")

        #btn = Button(window, text="Click Me", command=clicked)
        #btn.grid(column=2, row=0)

from threading import Thread
import cv2

class VideoGet:
    """
    Class that continuously gets frames from a VideoCapture object
    with a dedicated thread.
    """

    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        Thread(target=self.get, args=()).start()
        return self

    def get(self):
        while not self.stopped:
            if not self.grabbed:
                self.stop()
            else:
                (self.grabbed, self.frame) = self.stream.read()

    def stop(self):
        self.stopped = True


class VideoGUI(Frame):
    def __init__(self, parent, controller, subject_name, session):
        Frame.__init__(self, parent)
        self.controller = controller
        self.lmain = Label(self)
        self.subject_name = subject_name
        self.session = session
        #label = tk.Label(self, text="This is the start page", font=TITLE_FONT)
        self.lmain.pack(side="top", fill="x")
        #self.lmain.bind("<Button-1>", lambda e:self.controller.run_gui(["StartPage", "PageOne"]))
        self.side = LEFT
        self.col = 0
        self.counter = 0

        """
        button1 = tk.Button(self, text="Go to Page One",
                            command=lambda: controller.show_frame("PageOne"))
        button2 = tk.Button(self, text="Go to Page Two",
                            command=lambda: controller.show_frame("PageTwo"))
        button1.pack()
        button2.pack()
        """
        file_p_prefix = os.path.join(os.environ["REIT_HOME"], "BehavioralFeedbackEvaluator/resources/experiment/results")
        self.overall_result_img_fp = os.path.join(file_p_prefix, subject_name, session, "overall_results.png")

        self.image_path = os.path.join(RosPack().get_path("visu_ros_msg"), "include", "visu_ros_msg")
        if self.session == "robot":
            self.video_getter = VideoGet(0).start()
        self.s_time = time.time()
        self.end_time = 0
        self.video_stream()

    def video_stream(self):
        #print(time.time()-self.end_time)
        img = None
        if self.session == "tts":

            img = PIL.Image.open(os.path.join(self.image_path, "tts.png"))
            img = img.resize((self.winfo_width(), self.winfo_height()))
            '''
            self.counter = self.counter+1
            if self.counter == 60:
                print(time.time() - self.s_time)
                print("OKK")
                self.counter = 0
                self.session = "robot"
            '''
        elif self.session == "feedback":
            img = PIL.Image.open(self.overall_result_img_fp)
            img = img.resize((self.winfo_width(), self.winfo_height()))

        else:
            try:
                frame = self.video_getter.frame
                cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)

                img = Image.fromarray(cv2image) #.resize((self.winfo_width(), self.winfo_height()))
            except:
                pass

            '''
            img = PIL.Image.open(os.path.join(self.image_path, "framework.png"))
            img = img.resize((self.winfo_width(), self.winfo_height()))
            self.counter = self.counter + 1
            if self.counter == 60:
                self.counter = 0
                self.session = "tts"
            '''
        if img is not None:
            imgtk = ImageTk.PhotoImage(image=img)
            self.lmain.imgtk = imgtk
            self.lmain.configure(image=imgtk)

        self.lmain.after(1, self.video_stream)

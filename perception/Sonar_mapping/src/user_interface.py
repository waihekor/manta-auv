#!/usr/bin/python
import tkinter as tk
# Initialising and naming window
window = tk.Tk()
#window.geometry("1920x1080")
#window.resizable(0,0)
window.title('Calibration GUI')

# Creating interface
text1 = tk.Text(window, height=100, width=100)
place_holder_img = tk.PhotoImage(file='./SLAM_Harbor.png')
label = tk.Label(window, text = "Sonar-Camera Calibration").pack()
text1.insert(tk.END, '\n')
text1.image_create(tk.END, image=place_holder_img)
text1.pack(side=tk.LEFT)


text2 = tk.Text(window, height=100, width=100)
scroll = tk.Scrollbar(window, command=text2.yview)
text2.configure(yscrollcommand=scroll.set)
text2.tag_configure('bold_italics', font=('Arial', 12, 'bold', 'italic'))
text2.tag_configure('big', font=('Verdana', 20, 'bold'))
text2.tag_configure('color',
                    foreground='#476042',
                    font=('Tempus Sans ITC', 12, 'bold'))
text2.tag_bind('follow',
               '<1>',
               lambda e, t=text2: t.insert(tk.END, "Not now, maybe later!"))
text2.insert(tk.END,'\nWilliam Shakespeare\n', 'big')
quote = """
To be, or not to be that is the question:
Whether 'tis Nobler in the mind to suffer
The Slings and Arrows of outrageous Fortune,
Or to take Arms against a Sea of troubles,
"""
text2.insert(tk.END, quote, 'color')
text2.insert(tk.END, 'follow-up\n', 'follow')
text2.pack(side=tk.LEFT)
scroll.pack(side=tk.RIGHT, fill=tk.Y)

window.mainloop()
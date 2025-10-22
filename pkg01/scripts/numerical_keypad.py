import tkinter as tk
from tkinter import ttk

class NumericalKeypad:
    def __init__(self, root, callback=None):
        self.root = root
        self.root.title("Numerical Keypad")
        self.root.geometry("600x800")
        self.root.configure(bg='#667eea')
        
        self.current_input = ""
        self.submitted_values = []
        self.callback = callback  # Callback function to handle submitted input
        
        # Create main container
        self.container = tk.Frame(root, bg='white', padx=30, pady=30)
        self.container.pack(expand=True, fill='both', padx=20, pady=20)
        
        # Create display
        self.display = tk.Label(
            self.container,
            text="",
            font=('Arial', 24),
            bg='#f0f0f0',
            fg='#333',
            anchor='e',
            padx=20,
            pady=20,
            relief='solid',
            borderwidth=2
        )
        self.display.pack(fill='x', pady=(0, 20))
        
        # Create keypad frame
        self.keypad_frame = tk.Frame(self.container, bg='white')
        self.keypad_frame.pack()
        
        # Create number buttons
        buttons = [
            ('1', 0, 0), ('2', 0, 1), ('3', 0, 2),
            ('4', 1, 0), ('5', 1, 1), ('6', 1, 2),
            ('7', 2, 0), ('8', 2, 1), ('9', 2, 2),
        ]
        
        for (text, row, col) in buttons:
            btn = tk.Button(
                self.keypad_frame,
                text=text,
                font=('Arial', 20, 'bold'),
                bg='#667eea',
                fg='white',
                activebackground='#764ba2',
                activeforeground='white',
                width=5,
                height=2,
                relief='flat',
                cursor='hand2',
                command=lambda t=text: self.add_digit(t)
            )
            btn.grid(row=row, column=col, padx=5, pady=5)
        
        # Clear button
        clear_btn = tk.Button(
            self.keypad_frame,
            text='C',
            font=('Arial', 20, 'bold'),
            bg='#f5576c',
            fg='white',
            activebackground="#ff0000",
            activeforeground='white',
            width=5,
            height=2,
            relief='flat',
            cursor='hand2',
            command=self.clear_display
        )
        clear_btn.grid(row=0, column=3, padx=5, pady=5)
        
        # Undo button
        # undo_btn = tk.Button(
        #     self.keypad_frame,
        #     text='\u21a9',  # Undo symbol
        #     font=('Arial', 20, 'bold'),
        #     bg='#4facfe',
        #     fg='white',
        #     activebackground='#00f2fe',
        #     activeforeground='white',
        #     width=5,
        #     height=2,
        #     relief='flat',
        #     cursor='hand2',
        #     command=self.undo_last
        # )
        # undo_btn.grid(row=1, column=3, padx=5, pady=5)
        
        # # All button (clear all submitted values)
        # all_btn = tk.Button(
        #     self.keypad_frame,
        #     text='ALL',
        #     font=('Arial', 20, 'bold'),
        #     bg='#4facfe',
        #     fg='white',
        #     activebackground='#00f2fe',
        #     activeforeground='white',
        #     width=5,
        #     height=2,
        #     relief='flat',
        #     cursor='hand2',
        #     command=self.clear_all
        # )
        # all_btn.grid(row=2, column=3, padx=5, pady=5)
        
        # Zero button
        zero_btn = tk.Button(
            self.keypad_frame,
            text='0',
            font=('Arial', 20, 'bold'),
            bg='#667eea',
            fg='white',
            activebackground='#764ba2',
            activeforeground='white',
            width=5,
            height=2,
            relief='flat',
            cursor='hand2',
            command=lambda: self.add_digit('0')
        )
        zero_btn.grid(row=3, column=0, padx=5, pady=5)
        
        # Comma button
        comma_btn = tk.Button(
            self.keypad_frame,
            text='.',
            font=('Arial', 20, 'bold'),
            bg='#667eea',
            fg='white',
            activebackground='#764ba2',
            activeforeground='white',
            width=5,
            height=2,
            relief='flat',
            cursor='hand2',
            command=lambda: self.add_digit('.')
        )
        comma_btn.grid(row=3, column=1, padx=5, pady=5)

        # Enter button (spans 2 columns)
        enter_btn = tk.Button(
            self.keypad_frame,
            text='ENTER',
            font=('Arial', 20, 'bold'),
            bg="#4ffe92",
            fg='white',
            activebackground="#00fe15",
            activeforeground='white',
            width=11,
            height=2,
            relief='flat',
            cursor='hand2',
            command=self.submit_input
        )
        enter_btn.grid(row=3, column=2, columnspan=2, padx=5, pady=5)
        
        # Output section
        output_frame = tk.Frame(self.container, bg='#f9f9f9', relief='solid', borderwidth=1)
        output_frame.pack(fill='both', expand=True, pady=(20, 0))
        
        output_title = tk.Label(
            output_frame,
            text="Submitted Values:",
            font=('Arial', 12, 'bold'),
            bg='#f9f9f9',
            fg='#667eea',
            anchor='w'
        )
        output_title.pack(fill='x', padx=15, pady=(10, 5))
        
        # Create scrollable output list
        self.output_text = tk.Text(
            output_frame,
            font=('Arial', 11),
            bg='#f9f9f9',
            fg='#555',
            height=6,
            relief='flat',
            state='disabled'
        )
        self.output_text.pack(fill='both', expand=True, padx=15, pady=(0, 10))
        
        # Bind keyboard events
        self.root.bind('<Key>', self.on_key_press)
        
    def add_digit(self, digit):
        if digit == '.' and self.current_input.find('.') != -1:
            return
        self.current_input += digit
        self.update_display()
    
    def clear_display(self):
        self.current_input = ""
        self.update_display()
    
    def undo_last(self):
        """Remove the last submitted value and update the output list."""
        if self.submitted_values:
            removed_value = self.submitted_values.pop()
            print(f"UNDO: Removed value '{removed_value}'")
            self.update_output_list()
    
    def clear_all(self):
        """Submit a -1 meaning the calf has to drink all the milk"""
        self.current_input = "-1"
        self.submit_input()
    
    def update_display(self):
        self.display.config(text=self.current_input)
    
    def submit_input(self):
        if self.current_input:
            # particular cases
            # if it starts (ends) with ., make it starts (ends) with 0
            if self.current_input.startswith('.'):
                self.current_input = '0' + self.current_input
            elif self.current_input.endswith('.'):
                self.current_input += '0'

            self.submitted_values.append(self.current_input)
            
            # Print to console
            print(f"INPUT: {self.current_input}")
            
            # Call the callback function with the submitted value
            if self.callback:
                self.callback(self.current_input)
            
            # Update output list
            self.update_output_list()
            
            # Clear for next input
            self.current_input = ""
            self.update_display()
    
    def update_output_list(self):
        self.output_text.config(state='normal')
        self.output_text.delete('1.0', tk.END)
        
        # Show last 10 values in reverse order
        for value in reversed(self.submitted_values[-10:]):
            self.output_text.insert(tk.END, value + '\n')
        
        self.output_text.config(state='disabled')
    
    def on_key_press(self, event):
        if event.char.isdigit() or event.char == '.':
            self.add_digit(event.char)
        elif event.keysym == 'Return':
            self.submit_input()
        elif event.keysym == 'BackSpace':
            self.current_input = self.current_input[:-1]
            self.update_display()
        elif event.keysym == 'Delete':
            self.clear_display()
    
    def get_input(self):
        """
        Method to get the last submitted value.
        Returns None if no values have been submitted.
        """
        return self.submitted_values[-1] if self.submitted_values else None
    
    def get_all_inputs(self):
        """
        Method to get all submitted values.
        """
        return self.submitted_values.copy()

def main():
    # Define what happens when user submits input
    def on_input_received(value):
        print(f"Received value: {value}")
        # Do whatever you want with the value here
        # For example: process it, save it, use it in calculations, etc.
    
    root = tk.Tk()
    keypad = NumericalKeypad(root, callback=on_input_received)
    
    root.mainloop()

if __name__ == "__main__":
    main()
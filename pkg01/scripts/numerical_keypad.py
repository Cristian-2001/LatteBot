import tkinter as tk
from tkinter import ttk, messagebox

class CowMilkInterface:
    def __init__(self, root, callback=None, sequence_callback=None, used_cows_getter=None):
        self.root = root
        self.root.title("Milking Sequence Planner")
        
        # Set window size to 50% of screen dimensions
        self.root.update_idletasks()
        screen_width = root.winfo_screenwidth()
        screen_height = root.winfo_screenheight()

        width = int(screen_width * 0.4)
        height = int(screen_height * 1)
        x = (screen_width // 2) - (width // 2)
        y = (screen_height // 2) - (height // 2)
        self.root.geometry(f'{width}x{height}+{x}+{y}')
        
        self.root.configure(bg='#667eea')
        
        self.current_input = ""
        self.sequence = []  # List of tuples (cow_number, milk_quantity)
        self.callback = callback  # Callback for single addition
        self.sequence_callback = sequence_callback  # Callback for complete sequence
        self.used_cows_getter = used_cows_getter  # Function to get globally used cows
        
        # Create canvas with scrollbar for scrollable content
        self.canvas = tk.Canvas(root, bg='white', highlightthickness=0)
        self.scrollbar = tk.Scrollbar(root, orient="vertical", command=self.canvas.yview)
        self.scrollable_frame = tk.Frame(self.canvas, bg='white')
        
        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        )
        
        self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        
        # Pack canvas and scrollbar
        self.scrollbar.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)
        
        # Create main container inside scrollable frame
        self.container = tk.Frame(self.scrollable_frame, bg='white', padx=40, pady=40)
        self.container.pack(expand=True, fill='both', padx=20, pady=20)
        
        # Bind mousewheel scrolling
        self.canvas.bind_all("<MouseWheel>", self._on_mousewheel)
        
        # ===== COW SELECTION SECTION =====
        cow_frame = tk.Frame(self.container, bg='white')
        cow_frame.pack(fill='x', pady=(0, 15))
        
        cow_label = tk.Label(
            cow_frame,
            text="Select Cow:",
            font=('Arial', 12, 'bold'),
            bg='white',
            fg='#333'
        )
        cow_label.pack(anchor='w', pady=(0, 8))
        
        # Dropdown for cow selection
        self.cow_var = tk.StringVar()
        self.cow_dropdown = ttk.Combobox(
            cow_frame,
            textvariable=self.cow_var,
            values=[f"Cow {i}" for i in range(5)],  # Only 5 cows now (0-4)
            state='readonly',
            font=('Arial', 16),
            width=25
        )
        self.cow_dropdown.pack(fill='x')
        self.cow_dropdown.current(0)  # Set default selection
        
        # ===== MILK QUANTITY SECTION =====
        milk_frame = tk.Frame(self.container, bg='white')
        milk_frame.pack(fill='x', pady=(0, 15))
        
        milk_label = tk.Label(
            milk_frame,
            text="Liters of Milk per Cow: (max 10L per cow | max 10L total sequence)",
            font=('Arial', 11, 'bold'),
            bg='white',
            fg='#333'
        )
        milk_label.pack(anchor='w', pady=(0, 8))
        
        # Display for milk quantity
        self.display = tk.Label(
            milk_frame,
            text="0",
            font=('Arial', 24, 'bold'),
            bg='#f0f0f0',
            fg='#333',
            anchor='e',
            padx=15,
            pady=15,
            relief='solid',
            borderwidth=2
        )
        self.display.pack(fill='x', pady=(0, 15))
        
        # Create keypad frame
        self.keypad_frame = tk.Frame(milk_frame, bg='white')
        self.keypad_frame.pack()
        
        # Create number buttons (3x3 grid for 1-9)
        buttons = [
            ('1', 0, 0), ('2', 0, 1), ('3', 0, 2),
            ('4', 1, 0), ('5', 1, 1), ('6', 1, 2),
            ('7', 2, 0), ('8', 2, 1), ('9', 2, 2),
        ]
        
        for (text, row, col) in buttons:
            btn = tk.Button(
                self.keypad_frame,
                text=text,
                font=('Arial', 16, 'bold'),
                bg='#667eea',
                fg='white',
                activebackground='#764ba2',
                activeforeground='white',
                width=4,
                height=1,
                relief='flat',
                cursor='hand2',
                command=lambda t=text: self.add_digit(t)
            )
            btn.grid(row=row, column=col, padx=4, pady=4, sticky='nsew')
        
        # Zero button (spans 2 columns horizontally)
        zero_btn = tk.Button(
            self.keypad_frame,
            text='0',
            font=('Arial', 16, 'bold'),
            bg='#667eea',
            fg='white',
            activebackground='#764ba2',
            activeforeground='white',
            width=4,
            height=1,
            relief='flat',
            cursor='hand2',
            command=lambda: self.add_digit('0')
        )
        zero_btn.grid(row=3, column=0, columnspan=2, padx=4, pady=4, sticky='ew')
        
        # Decimal point button
        decimal_btn = tk.Button(
            self.keypad_frame,
            text='.',
            font=('Arial', 16, 'bold'),
            bg='#667eea',
            fg='white',
            activebackground='#764ba2',
            activeforeground='white',
            width=4,
            height=1,
            relief='flat',
            cursor='hand2',
            command=lambda: self.add_digit('.')
        )
        decimal_btn.grid(row=3, column=2, padx=4, pady=4, sticky='nsew')
        
        # Clear button (spans 2 rows vertically on the right)
        clear_btn = tk.Button(
            self.keypad_frame,
            text='C',
            font=('Arial', 18, 'bold'),
            bg='#f5576c',
            fg='white',
            activebackground="#ff0000",
            activeforeground='white',
            width=4,
            relief='flat',
            cursor='hand2',
            command=self.clear_display
        )
        clear_btn.grid(row=0, column=3, rowspan=2, padx=4, pady=4, sticky='nsew')
        
        # Add to sequence button (spans 2 rows vertically on the right)
        add_btn = tk.Button(
            self.keypad_frame,
            text='ADD',
            font=('Arial', 14, 'bold'),
            bg="#4ffe92",
            fg='white',
            activebackground="#00fe15",
            activeforeground='white',
            width=4,
            relief='flat',
            cursor='hand2',
            command=self.add_to_sequence
        )
        add_btn.grid(row=2, column=3, rowspan=2, padx=4, pady=4, sticky='nsew')
        
        # ===== SEQUENCE DISPLAY SECTION =====
        sequence_frame = tk.Frame(self.container, bg='#f9f9f9', relief='solid', borderwidth=2)
        sequence_frame.pack(fill='both', expand=True, pady=(15, 0))
        
        # Header with title and counter
        header_frame = tk.Frame(sequence_frame, bg='#667eea')
        header_frame.pack(fill='x')
        
        sequence_title = tk.Label(
            header_frame,
            text="Planned Sequence:",
            font=('Arial', 13, 'bold'),
            bg='#667eea',
            fg='white',
            anchor='w'
        )
        sequence_title.pack(side='left', padx=15, pady=8)
        
        self.sequence_counter = tk.Label(
            header_frame,
            text="0 cows",
            font=('Arial', 11, 'bold'),
            bg='#667eea',
            fg='white',
            anchor='e'
        )
        self.sequence_counter.pack(side='right', padx=15, pady=8)
        
        # Scrollable listbox with sequence
        list_frame = tk.Frame(sequence_frame, bg='#f9f9f9')
        list_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        # Create listbox with scrollbar
        scrollbar = tk.Scrollbar(list_frame)
        scrollbar.pack(side='right', fill='y')
        
        self.sequence_listbox = tk.Listbox(
            list_frame,
            font=('Arial', 11),
            bg='white',
            fg='#333',
            selectmode=tk.SINGLE,
            yscrollcommand=scrollbar.set,
            relief='flat',
            borderwidth=0,
            highlightthickness=1,
            highlightbackground='#ccc',
            activestyle='none'
        )
        self.sequence_listbox.pack(side='left', fill='both', expand=True)
        scrollbar.config(command=self.sequence_listbox.yview)
        
        # Button frame at bottom (only for editing sequence)
        button_frame = tk.Frame(sequence_frame, bg='#f9f9f9')
        button_frame.pack(fill='x', padx=10, pady=(0, 10))
        
        # Remove selected button
        remove_btn = tk.Button(
            button_frame,
            text='Remove Selected',
            font=('Arial', 11, 'bold'),
            bg='#f5576c',
            fg='white',
            activebackground='#ff0000',
            activeforeground='white',
            relief='flat',
            cursor='hand2',
            command=self.remove_selected
        )
        remove_btn.pack(side='left', padx=(0, 5), pady=5, fill='x', expand=True)
        
        # Clear all button
        clear_all_btn = tk.Button(
            button_frame,
            text='Clear All',
            font=('Arial', 11, 'bold'),
            bg='#ff8c42',
            fg='white',
            activebackground='#ff6600',
            activeforeground='white',
            relief='flat',
            cursor='hand2',
            command=self.clear_sequence
        )
        clear_all_btn.pack(side='left', padx=5, pady=5, fill='x', expand=True)
        
        # ===== BIG START BUTTON AT BOTTOM (inside container, still scrollable) =====
        start_frame = tk.Frame(self.container, bg='white')
        start_frame.pack(side='bottom', fill='x', padx=0, pady=(20, 0))
        
        # Start sequence button (BIG and PROMINENT at bottom)
        self.start_btn = tk.Button(
            start_frame,
            text='START SEQUENCE',
            font=('Arial', 18, 'bold'),
            bg='#00c853',
            fg='white',
            activebackground='#00e676',
            activeforeground='white',
            relief='raised',
            borderwidth=4,
            cursor='hand2',
            command=self.start_sequence,
            pady=20
        )
        self.start_btn.pack(fill='both', expand=True, padx=0, pady=0)
        
        # Bind keyboard events
        self.root.bind('<Key>', self.on_key_press)
    
    def _on_mousewheel(self, event):
        """Handle mousewheel scrolling"""
        self.canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        
    def add_digit(self, digit):
        # Prevent multiple decimal points
        if digit == '.' and self.current_input.find('.') != -1:
            return
        self.current_input += digit
        self.update_display()
    
    def clear_display(self):
        self.current_input = ""
        self.update_display()
    
    def update_display(self):
        display_text = self.current_input if self.current_input else "0"
        self.display.config(text=display_text)
    
    def add_to_sequence(self):
        """Add current cow and milk quantity to the sequence"""
        # Get selected cow number (extract number from "Cow X" or "Cow X [+]")
        cow_text = self.cow_var.get()
        cow_selection = int(cow_text.split()[1])  # Extract number from "Cow X" or "Cow X [+]"
        
        # Get milk quantity
        milk_quantity = self.current_input if self.current_input else "0"
        
        # Validate milk quantity
        try:
            milk_float = float(milk_quantity)
            if milk_float <= 0:
                messagebox.showwarning("Invalid Value", "Liters must be greater than zero!")
                return
            if milk_float > 10:
                messagebox.showwarning(
                    "Limit Exceeded", 
                    f"Maximum limit is 10 liters per cow!\nYou entered: {milk_float} liters"
                )
                return
        except ValueError:
            messagebox.showwarning("Invalid Value", "Please enter a valid number for liters!")
            return
        
        # Check total sequence limit (10L total for entire sequence)
        current_total = sum(float(liters) for _, liters in self.sequence)
        new_total = current_total + milk_float
        if new_total > 10:
            messagebox.showwarning(
                "Sequence Limit Exceeded",
                f"Maximum total for sequence is 10 liters!\n\n"
                f"Current total: {current_total:.1f}L\n"
                f"Trying to add: {milk_float:.1f}L\n"
                f"Would be: {new_total:.1f}L (exceeds 10L limit)\n\n"
                f"Remove some cows or reduce quantities."
            )
            return
        
        # Check if cow is already in current sequence
        existing_cows = [cow_num for cow_num, _ in self.sequence]
        if cow_selection in existing_cows:
            messagebox.showwarning(
                "Cow Already Added",
                f"Cow {cow_selection} is already in the current sequence!\n\n"
                f"Each cow can only be added once per sequence.\n"
                f"Remove the cow from the list first if you want to change the liters."
            )
            return
        
        # Check if cow was used in previous sequences (global check)
        if self.used_cows_getter:
            globally_used_cows = self.used_cows_getter()
            if cow_selection in globally_used_cows:
                messagebox.showerror(
                    "Cow Already Processed",
                    f"Cow {cow_selection} was already processed in a previous sequence!\n\n"
                    f"You cannot reuse cows that have been started in previous sequences.\n\n"
                    f"Globally used cows: {sorted(globally_used_cows)}"
                )
                return
        
        # Handle edge cases for decimal point
        if milk_quantity.startswith('.'):
            milk_quantity = '0' + milk_quantity
        elif milk_quantity.endswith('.'):
            milk_quantity += '0'
        
        # Add to sequence
        self.sequence.append((cow_selection, milk_quantity))
        
        # Call the callback function for single addition
        if self.callback:
            self.callback(cow_selection, milk_quantity)
        
        # Update display
        self.update_sequence_display()
        
        # Clear milk input for next entry
        self.current_input = ""
        self.update_display()
        
        # Show confirmation
        print(f"Added: Cow {cow_selection} - {milk_quantity} liters")
        
        # Update available cows in dropdown (optional: highlight used cows)
        self.update_cow_dropdown_colors()
    
    def update_sequence_display(self):
        """Update the listbox showing the sequence"""
        # Clear listbox
        self.sequence_listbox.delete(0, tk.END)
        
        # Add all items in sequence
        for idx, (cow_num, liters) in enumerate(self.sequence, 1):
            item_text = f"{idx}. Cow {cow_num} → {liters} liters"
            self.sequence_listbox.insert(tk.END, item_text)
        
        # Calculate total liters
        total_liters = sum(float(liters) for _, liters in self.sequence)
        
        # Update counter with total liters
        count = len(self.sequence)
        counter_text = f"{count} cow" if count == 1 else f"{count} cows"
        counter_text += f" | {total_liters:.1f}L / 10L"
        self.sequence_counter.config(text=counter_text)
    
    def update_cow_dropdown_colors(self):
        """Update dropdown to show which cows are in the current sequence and globally used"""
        # Get cows in current sequence
        used_in_sequence = [cow_num for cow_num, _ in self.sequence]
        
        # Get globally used cows from previous sequences
        globally_used = []
        if self.used_cows_getter:
            globally_used = self.used_cows_getter()
        
        # Update dropdown values with indicators
        new_values = []
        for i in range(5):  # Only 5 cows now (0-4)
            if i in globally_used:
                # Cow was used in previous sequences (cannot be selected)
                new_values.append(f"Cow {i} [X]")
            elif i in used_in_sequence:
                # Cow is in current sequence
                new_values.append(f"Cow {i} [+]")
            else:
                # Cow is available
                new_values.append(f"Cow {i}")
        
        self.cow_dropdown['values'] = new_values
        
        # Reset selection to first available cow (not globally used, not in sequence)
        for i in range(5):  # Only 5 cows now (0-4)
            if i not in globally_used and i not in used_in_sequence:
                self.cow_dropdown.current(i)
                break
    
    def remove_selected(self):
        """Remove selected item from sequence"""
        selection = self.sequence_listbox.curselection()
        if selection:
            idx = selection[0]
            removed = self.sequence.pop(idx)
            self.update_sequence_display()
            self.update_cow_dropdown_colors()
            print(f"Removed: Cow {removed[0]} - {removed[1]} liters")
        else:
            messagebox.showinfo("No Selection", "Select an item from the list to remove")
    
    def clear_sequence(self):
        """Clear entire sequence"""
        if self.sequence:
            response = messagebox.askyesno(
                "Confirm Clear",
                f"Do you want to clear the entire sequence ({len(self.sequence)} cows)?"
            )
            if response:
                self.sequence.clear()
                self.update_sequence_display()
                self.update_cow_dropdown_colors()
                print("Sequence completely cleared")
        else:
            messagebox.showinfo("Empty Sequence", "There are no items to clear")
    
    def start_sequence(self):
        """Start the robot sequence with all planned cows"""
        if not self.sequence:
            messagebox.showwarning("Empty Sequence", "Add at least one cow to the sequence!")
            return
        
        # Confirm before starting
        response = messagebox.askyesno(
            "Start Sequence",
            f"Start sequence with {len(self.sequence)} cows?\n\n" +
            "\n".join([f"Cow {cow} → {liters}L" for cow, liters in self.sequence])
        )
        
        if response:
            print("\n" + "="*50)
            print("STARTING MILKING SEQUENCE")
            print("="*50)
            
            # Call sequence callback with complete sequence
            if self.sequence_callback:
                self.sequence_callback(self.sequence.copy())
            
            # Print sequence details
            for idx, (cow_num, liters) in enumerate(self.sequence, 1):
                print(f"  {idx}. Cow {cow_num} -> {liters} liters")
            
            print("="*50 + "\n")
            
            # Clear the current sequence after starting
            # (cows are now tracked globally by pickup_site)
            self.sequence.clear()
            self.update_sequence_display()
            self.update_cow_dropdown_colors()
            
            # Show success message
            messagebox.showinfo(
                "Sequence Started",
                f"Sequence sent to robot!\n\n"
                f"You can now plan a new sequence with remaining cows."
            )
    
    def submit_input(self):
        """Legacy method - redirects to add_to_sequence"""
        self.add_to_sequence()
    
    def on_key_press(self, event):
        if event.char.isdigit() or event.char == '.':
            self.add_digit(event.char)
        elif event.keysym == 'Return':
            self.add_to_sequence()
        elif event.keysym == 'BackSpace':
            self.current_input = self.current_input[:-1]
            self.update_display()
        elif event.keysym == 'Delete':
            self.clear_display()
    
    def get_sequence(self):
        """
        Method to get the complete sequence.
        Returns list of tuples: [(cow_number, milk_quantity), ...]
        """
        return self.sequence.copy()
    
    def get_input(self):
        """
        Legacy method - returns last item in sequence if exists.
        """
        if self.sequence:
            cow, liters = self.sequence[-1]
            return f"Cow {cow}: {liters} liters"
        return None
    
    def get_all_inputs(self):
        """
        Legacy method - returns all sequence items as formatted strings.
        """
        return [f"Cow {cow}: {liters} liters" for cow, liters in self.sequence]


def main():
    # Define what happens when user adds a single cow
    def on_cow_added(cow_number, milk_quantity):
        print(f"Added - Cow: {cow_number}, Liters: {milk_quantity}")
    
    # Define what happens when user starts the complete sequence
    def on_sequence_start(sequence):
        print("\nROBOT RECEIVES SEQUENCE:")
        print("-" * 40)
        for idx, (cow_num, liters) in enumerate(sequence, 1):
            print(f"  Step {idx}: Milk Cow {cow_num} -> {liters} liters")
        print("-" * 40)
        print(f"Total operations: {len(sequence)}")
        print()
        
        # Here you can add logic to send sequence to ROS robot
        # Example:
        # robot_controller.execute_milking_sequence(sequence)
    
    root = tk.Tk()
    interface = CowMilkInterface(
        root, 
        callback=on_cow_added,
        sequence_callback=on_sequence_start
    )
    root.mainloop()


if __name__ == "__main__":
    main()
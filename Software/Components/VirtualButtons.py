import cv2
import sys

class VirtualButtons:
    def __init__(self, frame, landmarks=None, button_configs=None):
        self.frame = frame
        self.landmarks = landmarks or []
        self.buttons = []
        self._should_exit = False
        
        # Get frame dimensions
        self.frame_height, self.frame_width = frame.shape[:2]
        
        # Default buttons positioned on the right side, stacked vertically
        button_width, button_height = 120, 60
        right_margin = 20
        vertical_spacing = 20
        start_y = 50  # Higher up position
        
        default_configs = [
            {'pos': (self.frame_width - button_width - right_margin, start_y), 
             'size': (button_width, button_height), 
             'label': 'Option 1', 
             'action': lambda: print("Option 1 selected")},
             
            {'pos': (self.frame_width - button_width - right_margin, start_y + button_height + vertical_spacing), 
             'size': (button_width, button_height), 
             'label': 'Option 2', 
             'action': lambda: print("Option 2 selected")}
        ]
        
        self.button_configs = button_configs or default_configs
        self.setup_buttons()
        self.add_exit_button()
        
    def exit_action(self):
        """Clean exit action that sets the exit flag"""
        print("Exit requested - cleaning up")
        self._should_exit = True
        cv2.destroyAllWindows()
        sys.exit(0)
        
    def should_exit(self):
        return self._should_exit
        
    def reset_exit(self):
        self._should_exit = False
        
    def setup_buttons(self):
        for config in self.button_configs:
            self.add_button(
                pos=config['pos'],
                label=config['label'],
                action=config.get('action'),
                size=config.get('size', (120, 60)),
                color=config.get('color', (100, 200, 100)),  # Light green
                active_color=config.get('active_color', (50, 150, 50))  # Darker green
            )
    
    def draw_buttons(self):
        for button in self.buttons:
            x, y = button['pos']
            w, h = button['size']
            
            # Draw button with rounded corners
            self.draw_rounded_rect(x, y, w, h, 15, button['color'])
            
            # Draw label (centered)
            text_size = cv2.getTextSize(button['label'], cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)[0]
            text_x = x + (w - text_size[0]) // 2
            text_y = y + (h + text_size[1]) // 2
            cv2.putText(self.frame, button['label'], (text_x, text_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        return self.frame
    
    def draw_rounded_rect(self, x, y, w, h, radius, color):
        """Helper function to draw rounded rectangles"""
        cv2.rectangle(self.frame, (x + radius, y), (x + w - radius, y + h), color, -1)
        cv2.rectangle(self.frame, (x, y + radius), (x + w, y + h - radius), color, -1)
        cv2.circle(self.frame, (x + radius, y + radius), radius, color, -1)
        cv2.circle(self.frame, (x + w - radius, y + radius), radius, color, -1)
        cv2.circle(self.frame, (x + radius, y + h - radius), radius, color, -1)
        cv2.circle(self.frame, (x + w - radius, y + h - radius), radius, color, -1)
        
    def check_button_presses(self, finger_tip_index=8):
        if not self.landmarks or len(self.landmarks) <= finger_tip_index:
            return False
        
        finger_x, finger_y = self.landmarks[finger_tip_index]
        any_pressed = False
        
        for button in self.buttons:
            btn_x, btn_y = button['pos']
            btn_w, btn_h = button['size']
            
            if (btn_x <= finger_x <= btn_x + btn_w and 
                btn_y <= finger_y <= btn_y + btn_h):
                button['color'] = button['active_color']
                if not button['pressed']:
                    button['action']()
                    button['pressed'] = True
                any_pressed = True
            else:
                button['color'] = button.get('original_color', (100, 200, 100))
                button['pressed'] = False
                
        return any_pressed
    
    def add_button(self, pos, label, action=None, size=(120, 60), color=None, active_color=None):
        default_color = (100, 200, 100)  # Light green
        default_active = (50, 150, 50)   # Darker green
        
        self.buttons.append({
            'pos': pos,
            'size': size,
            'label': label,
            'color': color if color else default_color,
            'original_color': color if color else default_color,
            'active_color': active_color if active_color else default_active,
            'action': action or (lambda: print(f"{label} pressed")),
            'pressed': False
        })
    
    def add_exit_button(self, pos=None, size=(80, 40)):
        """Add exit button at top-right corner"""
        if pos is None:
            pos = (self.frame_width - size[0] - 10, 10)
        
        self.add_button(
            pos=pos,
            size=size,
            label="X",
            color=(50, 50, 200),  # Blue
            active_color=(30, 30, 150),  # Darker blue
            action=self.exit_action
        )
    
    def clear_buttons(self):
        self.buttons = []
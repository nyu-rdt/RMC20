import pygame

# class to manage key events using pygame
class KeyManager:
    def __init__(self, title="clicky clack", width=640, height=480):
        # initializing pygame window
        self.width = width
        self.height = height
        self.gl_setup(title, width, height)

        # setting initial values for wasd
        self.w = 0
        self.a = 0
        self.s = 0
        self.d = 0

    # function used to update and check for pygame events
    def iter(self):
        # checking each event
        for event in pygame.event.get():
            if event.type == pygame.NOEVENT: # no event
                break
            elif event.type == pygame.QUIT: return # escape key
            elif event.type == pygame.KEYDOWN: # key pressed
                press = chr(event.key)
                if press == 'w': 
                elif press == 'a': self.a = 1
                elif press == 's': self.s = 1
                elif press == 'd': self.d = 1 
            elif event.type == pygame.KEYUP: # key released
                release = chr(event.key)
                if release == 'w': self.w = 0
                elif release == 'a': self.a = 0
                elif release == 's': self.s = 0
                elif release == 'd': self.d = 0
    
    # function used to set up a pygame window
    # title: title of window
    # width: width of window
    # height: height of window
    def gl_setup(self, title, width, height):
        pygame.init()
        screen_size = (width, height)
        screen = pygame.display.set_mode(screen_size)

    def __del__(self):
        pygame.quit()


# def main():
#     man = KeyManager()
#     man.loop()

# main()

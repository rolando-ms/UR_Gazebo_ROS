#!/usr/bin/env/ python


# Based on http://thepythongamebook.com/en:pygame:step002
# This script runs a pygame window that is used to read the keystrokes

import pygame
import time


class PygWindow(object):

    def __init__(self, width=640, height=480, refresh_rate=8):
        pygame.init()
        pygame.display.set_caption("Press ESC to quit")
        self.width = width
        self.height = height
        self.refresh_rate = refresh_rate
        self.screen = pygame.display.set_mode((self.width, self.height), pygame.DOUBLEBUF)
        self.background = pygame.Surface(self.screen.get_size()).convert()

    # This Function opens a pygame window that captures keystrokes. Events are handled with pump()

    def KeyPolling(self, queueInput):
        running = True
        while running:
            keys = pygame.key.get_pressed()
            queueInput.put(pygame.key.get_pressed())
            pygame.event.pump()  # This is only used to handle events if there is not another event handler
            if keys[pygame.K_SPACE] or keys[pygame.K_ESCAPE]:
                running = False
            pygame.event.clear()
            pygame.time.wait(self.refresh_rate)     # Time is in milliseconds
            #time.sleep(0.008)
            with queueInput.mutex:
                queueInput.queue.clear()

        pygame.quit()

    # This Function opens a pygame window that captures keystrokes. Events are handled with event.get()

    def KeyPolling2(self, queueInput):
        running = True
        key_pressed = None
        while running:

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                if event.type == pygame.KEYDOWN:
                    key_pressed = event.key
                if event.type == pygame.KEYUP:
                    if event.key == key_pressed:
                        key_pressed = None
            if key_pressed:
                queueInput.put(key_pressed)
                if key_pressed == pygame.K_SPACE or key_pressed == pygame.K_ESCAPE:
                    running = False
                print key_pressed
        pygame.quit()

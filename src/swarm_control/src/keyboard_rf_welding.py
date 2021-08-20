#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

import pygame

'''
keyboard_rf_welding.py
Alex Elias

Sends velocity commands to swarm controller based on arrow keys

Parameters:
	speed_x: Speed in the x direction (up/down keys)
	speed_y: Speed in the y direction (left/right keys)
	vel_topic_name: ROS topic to send Twist messages to
'''

class Keyboard_RF_welding():
	def __init__(self):
		rospy.init_node('keyboard_rf_welding', anonymous=False)

		# Read in paramters
		self.speed_x = rospy.get_param('~speed_x')
		self.speed_y = rospy.get_param('~speed_y')
		vel_topic_name = rospy.get_param('~vel_topic_name')

		# Initialize
		pygame.init()
		self.screen = pygame.display.set_mode((640, 480), pygame.RESIZABLE)
		pygame.display.set_caption('Keyboard RF Welding')

		self.last_vel = [0.0, 0.0]

		# Publish
		self.vel_pub = rospy.Publisher(vel_topic_name, Twist, queue_size=1)


	def process_keys(self, keys):
		u = keys[pygame.K_UP]
		d = keys[pygame.K_DOWN]
		l = keys[pygame.K_LEFT]
		r = keys[pygame.K_RIGHT]
		s = keys[pygame.K_SPACE]

		v_x, v_y = 0, 0

		output_enabled = s
		if(output_enabled):
			text = '0'
		else:
			text = 'X'

		if (u + d + l + r) == 1 and output_enabled:
			if u:
				v_x, v_y = self.speed_x, 0
				text = u"\N{Upwards Arrow}"
			elif d:
				v_x, v_y = -self.speed_x, 0
				text = u"\N{Downwards Arrow}"
			elif l:
				v_x, v_y = 0, self.speed_y
				text = u"\N{Leftwards Arrow}"
			elif r:
				v_x, v_y = 0, -self.speed_y
				text = u"\N{Rightwards Arrow}"

		# If we disable output, send one message of [0,0] so the robots stop
		if output_enabled:
			self.send_vel(v_x, v_y)
			self.last_vel = [v_x, v_y]
		elif not self.last_vel == [0.0, 0.0]:
			self.send_vel(0.0, 0.0)
			self.last_vel = [0.0, 0.0]

		self.update_screen_text(text)
	


	def update_screen_text(self, text):
		white = (255,255,255)
		black = (0,0,0)

		w, h = pygame.display.get_surface().get_size()

		font = pygame.font.SysFont("Arial",int(h*0.75))

		text_render = font.render(text, True, white)

		textRect = text_render.get_rect()
		textRect.center = (w // 2, h // 2)

		self.screen.fill(black)
		self.screen.blit(text_render, textRect)
		pygame.display.update()


	def send_vel(self, v_x, v_y):
		msg = Twist()
		msg.linear.x = v_x
		msg.linear.y = v_y
		self.vel_pub.publish(msg)


	def run(self):
		while not rospy.is_shutdown():
			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					pygame.quit()
					return
			keys = pygame.key.get_pressed()
			self.process_keys(keys)
			rospy.sleep(0.01)


if __name__ == '__main__':
	keyboard_RF_welding = Keyboard_RF_welding()
	keyboard_RF_welding.run()
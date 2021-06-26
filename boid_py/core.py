
import numpy as np
import cupy as cp

class boid:
	"""
	Class for calculating the positions of boids
	"""
	def __init__(self, num_boid=100):

		# attributes:
		self.N = num_boid
		# strength of force
		self.cohesion_force = 0.0008
		self.separation_force = 0.15
		self.alignment_force = 1.0

		# distance at force-induction
		self.cohesion_distance = 0.8
		self.separation_distance = 0.08
		self.alignment_distance = 0.1
		# angle of force
		self.cohesion_angle = np.pi/2
		self.separation_angle = np.pi/2
		self.alignment_angle = np.pi/3
		# max/min of speed
		self.min_vel = 0.0005
		self.max_vel = 0.1
		# force at boundary
		self.boundary_force = 0.001

		# array for positions and distance
		self.x = cp.random.rand(self.N, 3)*2 - 1
		self.v = (cp.random.rand(self.N, 3)*2 - 1)*self.min_vel
		self.r = cp.empty((self.N, 3))

		# array for three forces
		self.dv_coh = cp.empty((self.N, 3))
		self.dv_sep = cp.empty((self.N, 3))
		self.dv_ali = cp.empty((self.N, 3))
		self.dv_boundary = cp.empty((self.N, 3))

	def update(self):
		for i in range(self.N):
			x_this = self.x[i]
			v_this = self.v[i]
			
			# x_that = cp.delete(self.x, i, axis=0)
			# v_that = cp.delete(self.v, i, axis=0)

			x_that = self.x
			v_that = self.v
			
			distance = cp.linalg.norm(x_that - x_this, axis=1)
			angle = cp.arccos(cp.dot(v_this, (x_that-x_this).T) / (np.linalg.norm(v_this) * \
				    cp.linalg.norm((x_that-x_this), axis=1)))
			
			coh_agents_x = x_that[ (distance < self.cohesion_distance) & (angle < self.cohesion_angle)]
			sep_agents_x = x_that[ (distance < self.separation_distance) & (angle < self.separation_angle)]
			ali_agents_x = v_that[ (distance < self.alignment_distance) & (angle < self.alignment_angle)]

			self.dv_coh[i] = self.cohesion_force * (cp.average(coh_agents_x, axis=0) - x_this) if (len(coh_agents_x) > 0) else 0
			self.dv_sep[i] = self.separation_force * (cp.average(sep_agents_x, axis=0) - x_this) if (len(sep_agents_x) > 0) else 0
			self.dv_ali[i] = self.alignment_force * (cp.average(coh_agents_x, axis=0) - v_this) if (len(ali_agents_x) > 0) else 0
			
			dist_center = cp.linalg.norm(x_this)
			self.dv_boundary[i] = - self.boundary_force * x_this * (dist_center - 1) / dist_center if (dist_center > 1) else 0

		self.v += self.dv_coh + self.dv_sep + self.dv_ali + self.dv_boundary
		for i in range(self.N):
			v_abs = cp.linalg.norm(self.v[i])
			if (v_abs < self.min_vel):
				self.v[i] = self.min_vel * self.v[i] / v_abs
			elif (v_abs > self.max_vel):
				self.v[i] = self.max_vel * self.v[i] / v_abs

		self.x += self.v
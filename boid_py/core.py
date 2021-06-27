
import numpy as np

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
		self.x = np.random.rand(self.N, 3)*2 - 1
		self.v = (np.random.rand(self.N, 3)*2 - 1)*self.min_vel
		self.r = np.empty((self.N, 3))

		# array for tensor calculation
		self.diff_x = np.zeros((self.N, self.N, 3))
		self.distance = np.empty((self.N, self.N))
		self.angle = np.empty((self.N, self.N))
		
		# array for three forces
		self.dv_coh = np.empty((self.N, 3))
		self.dv_sep = np.empty((self.N, 3))
		self.dv_ali = np.empty((self.N, 3))
		self.dv_boundary = np.empty((self.N, 3))

	def update(self):
		self.diff_x *= 0.0
		self.diff_x += self.x.reshape((-1, self.N, 3))
		self.diff_x -= self.x.reshape((self.N, -1, 3))

		for i in range(self.N):
			x_this = self.x[i]
			v_this = self.v[i]
			
			# x_that = np.delete(self.x, i, axis=0)
			# v_that = np.delete(self.v, i, axis=0)
			x_that = self.x
			v_that = self.v

			# self.diff_x[i] = x_that - x_this
			
			self.distance[i] = np.linalg.norm(self.diff_x[i], axis=1)
			self.angle[i] = np.arccos(np.dot(v_this, (self.diff_x[i]).T) / (np.linalg.norm(v_this) * \
				    np.linalg.norm((self.diff_x[i]), axis=1)))
			
			coh_agents_x = x_that[ (self.distance[i] < self.cohesion_distance) &   (self.angle[i] < self.cohesion_angle)]
			sep_agents_x = x_that[ (self.distance[i] < self.separation_distance) & (self.angle[i] < self.separation_angle)]
			ali_agents_v = v_that[ (self.distance[i] < self.alignment_distance) &  (self.angle[i] < self.alignment_angle)]

			self.dv_coh[i] = self.cohesion_force * (np.average(coh_agents_x, axis=0) - x_this) if (len(coh_agents_x) > 0) else 0
			self.dv_sep[i] = self.separation_force * (np.average(sep_agents_x, axis=0) - x_this) if (len(sep_agents_x) > 0) else 0
			self.dv_ali[i] = self.alignment_force * (np.average(ali_agents_v, axis=0) - v_this) if (len(ali_agents_v) > 0) else 0
			
			dist_center = np.linalg.norm(x_this)
			self.dv_boundary[i] = - self.boundary_force * x_this * (dist_center - 1) / dist_center if (dist_center > 1) else 0

		self.v += self.dv_coh + self.dv_sep + self.dv_ali + self.dv_boundary
		for i in range(self.N):
			v_abs = np.linalg.norm(self.v[i])
			if (v_abs < self.min_vel):
				self.v[i] = self.min_vel * self.v[i] / v_abs
			elif (v_abs > self.max_vel):
				self.v[i] = self.max_vel * self.v[i] / v_abs

		self.x += self.v
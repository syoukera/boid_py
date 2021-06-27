
import numpy as np

class boid:
	"""
	Class for calculating the positions of boids
	"""
	def __init__(self, num_boid=100):

		# attributes:
		self.N = num_boid
		# strength of force
		self.cohesion_force = 0.08
		self.separation_force = 1.15
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

		self.coh_agents_x = np.empty((self.N, self.N, 3))
		self.sep_agents_x = np.empty((self.N, self.N, 3))
		self.ali_agents_v = np.empty((self.N, self.N, 3))
		
		# array for three forces
		self.dv_coh = np.empty((self.N, 3))
		self.dv_sep = np.empty((self.N, 3))
		self.dv_ali = np.empty((self.N, 3))
		self.dv_boundary = np.empty((self.N, 3))

	def update(self):
		'''
		Update positions of boids
		1. Calculate distance and angle for every pairs of bois 
		2. Calculate dv for Cohesion, Separation, Alignment, and Boundary
		3. Add dv to v
		4. Add v to x
		'''
		self.diff_x *= 0.0
		self.diff_x += self.x.reshape((-1, self.N, 3))
		self.diff_x -= self.x.reshape((self.N, -1, 3))

		self.distance = np.linalg.norm(self.diff_x, axis=2)
		self.angle = np.arccos(
						np.divide(
							np.sum(np.multiply(self.v, self.diff_x), axis=2),
							np.multiply(np.linalg.norm(self.v, axis=1) , np.linalg.norm(self.diff_x, axis=2))
						)
					)

		# Cohesion
		coh_agents_bool = (self.distance > self.cohesion_distance) | (self.angle > self.cohesion_angle)

		self.coh_agents_x *= 0.0
		self.coh_agents_x += self.x
		self.coh_agents_x[coh_agents_bool] = 0.0
		
		coh_agents_num = coh_agents_bool.shape[1] - np.count_nonzero(coh_agents_bool, axis=1)
		coh_agents_num[coh_agents_num == 0] = 1
		
		self.dv_coh = self.cohesion_force*(np.divide(np.sum(self.coh_agents_x, axis=1).T, coh_agents_num).T - self.x)
		
		# Separation
		sep_agents_bool = (self.distance > self.separation_distance) | (self.angle > self.separation_angle)

		self.sep_agents_x *= 0.0
		self.sep_agents_x += self.x
		self.sep_agents_x[sep_agents_bool] = 0.0
		
		sep_agents_num = sep_agents_bool.shape[1] - np.count_nonzero(sep_agents_bool, axis=1)
		sep_agents_num[sep_agents_num == 0] = 1
		
		self.dv_sep = self.separation_force*(np.divide(np.sum(self.sep_agents_x, axis=1).T, sep_agents_num).T - self.x)

		# Alignment
		ali_agents_bool = (self.distance > self.alignment_distance) | (self.angle > self.alignment_angle)

		self.ali_agents_v *= 0.0
		self.ali_agents_v += self.v
		self.ali_agents_v[ali_agents_bool] = 0.0
		
		ali_agents_num = ali_agents_bool.shape[1] - np.count_nonzero(ali_agents_bool, axis=1)
		ali_agents_num[ali_agents_num == 0] = 1
		
		self.dv_ali = self.alignment_force*(np.divide(np.sum(self.ali_agents_v, axis=1).T, ali_agents_num).T - self.v)

		# Boundary
		self.dist_center = np.linalg.norm(self.x, axis=1)
		dist_center_bool = (self.dist_center < 1)
		self.dv_boundary = - self.boundary_force*np.multiply(
				self.x.T, 
				np.divide(self.dist_center-1, self.dist_center)
			).T
		self.dv_boundary[dist_center_bool] = 0.0

		# Update v
		self.v += self.dv_coh + self.dv_sep + self.dv_ali + self.dv_boundary
		
		v_abs = np.linalg.norm(self.v, axis=1)

		min_vel_bool = (v_abs < self.min_vel)
		self.v[min_vel_bool] = self.min_vel*np.divide(self.v[min_vel_bool].T, v_abs[min_vel_bool]).T
		max_vel_bool = (v_abs > self.max_vel)
		self.v[max_vel_bool] = self.max_vel*np.divide(self.v[max_vel_bool].T, v_abs[max_vel_bool]).T

		self.x += self.v
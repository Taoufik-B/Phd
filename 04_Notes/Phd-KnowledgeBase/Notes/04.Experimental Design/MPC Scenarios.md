Varying MPC internals
- dT : Discretization step
- N : Receding Horizon
- Q : States running cost
- R : Controls running cost


Configuration example
> [!code]
> 
	```yaml
	NMPC.internals:
	dT: 0.1 # This is a comment
	N: 10 # Horizon
	Q: # Stage cost
	- 10 # x
	- 10 # y
	- 0.5 # psi heading angle
	- 0.01 # delta steering angle
	R:
	- 0.5 # velocity
	- 0.05 # steering rate
	only_euler: True #use rk4  
	NMPC.externals:
	u_ref:
	- 20 # v (m/s)
	- 0 # phi steering rate (rad/s2)
	vehicle:
	model_type: rac # (rac, fac, cog) rear axle center
	L: 3.0 # vehicle length
	Lr: 1.382 # distance from rac to cog
	bounds:
	x:	
	- [-20000, -20000, -3.14, -1.22] #lbx
	- [20000, 200000, 3.14, 1.22] #ubx
	u:
	- [0, -0.78] #lbu
	- [25, 0.78] #ubu
	g:
	- [0, 0, 0, 0] #lbg
	- [0, 0, 0, 0] #ubg  
	NMPC.environment:
	world_map: Town05 #example carla map
	trajectory:
	wps_file: ./maps/carla_town05_02012024.wps #wps file name env_mapname_date.wps (npy form)
	wps_factor: 4 # reduce the total number of waypoints by a facotr of 4
	run_environment: Simulation # or Carla simulation```

- [ ] Finalize the visualization
	- [ ] What are the elements that needs to be plotted
	- [ ] 
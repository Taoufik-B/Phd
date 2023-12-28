#%%
begin
  import Pkg
  Pkg.activate(".")
end
#%%
begin
  using JuMP, Ipopt
  using Plots
  using LinearAlgebra
  using NPZ
  using PyCall
end
#%%
#### Carla envs
begin
  carla = pyimport("carla")
  client = carla.Client("192.168.11.110",2000)
  client.set_timeout(20)
  world = client.load_world("Town05")
  settings = world.get_settings()
  settings.fixed_delta_seconds = .05
  settings.synchronous_mode = true
  settings.substepping = true
  settings.max_substep_delta_time = 0.005
  settings.max_substeps = 10
  world.apply_settings(settings)
  veh_bp = world.get_blueprint_library().filter("*veh*tesla*3*")[1]
  spawn_points = world.get_map().get_spawn_points()
  veh_spawn_point = spawn_points[rand(1:length(spawn_points),1)...]
  try
    global ego = world.spawn_actor(veh_bp, veh_spawn_point)
    println("Ego actor spawned successfly.")
  catch e
    println(e)
  end
end
#%%
begin
  function get_ego_view()
    spec_tr = cam.get_transform()
    spectator.set_transform(spec_tr)
    world.tick()
  end

  function get_ego_status()
    transform = ego.get_transform()
    x = transform.location.x + l1.x
    y = transform.location.y + l1.y
    ψ = transform.rotation.yaw 
    delta = ego.get_control().steer
    return [x,y, ψ, delta]
  end

  function set_ego_location(x_ref)
    loc = carla.Location(x=x_ref[1],y=x_ref[2])
    rot = carla.Rotation(yaw=x_ref[3], roll=0, pitch=0)
    ego.set_transform(carla.Transform(loc,rot))
    world.tick()
  end

  function attach_cam2ego()
    cam_bp = world.get_blueprint_library().find("sensor.camera.rgb")
    loc = carla.Location(x=-2,z=15)
    rot = carla.Rotation(pitch=-60)
    transform = carla.Transform(loc,rot)
    return world.spawn_actor(cam_bp, transform, attach_to=ego)
  end
  shift(st,con)=st + dT * [cos(st[3]+st[4]) 0;sin(st[3]+st[4]) 0; sin(st[4])/L 0; 0 1]*con
end
#%%
spectator = world.get_spectator()
cam = attach_cam2ego()
#%%
begin
  dT = 0.05
  N = 10
  v_max = 5.0
  v_min = 0.0
  v_ref = v_max

  phi_max = deg2rad(45)
  phi_min = -phi_max
end
begin
  xref = (npzread("wps.npy") |> permutedims)[:,1:4:end]
  phy_ctrl = ego.get_physics_control()
  transform = ego.get_transform()
  forward, right, up = transform.get_forward_vector(), transform.get_right_vector(), transform.get_up_vector()
  wh11, wh12, _, _ = phy_ctrl.wheels
  front = carla.Vector3D((wh11.position.x + wh12.position.x)/2, wh11.position.y, wh11.position.z)/100
  l1 = carla.Vector3D((front-ego.get_location()).dot(forward),(front-ego.get_location()).dot(right),(front-ego.get_location()).dot(up))
  L = wh11.position.distance(phy_ctrl.wheels[3].position)/100
  #xref[3,findall(xref[3,:].<-3.1)] *= -1.0
  xref[3,:] = deg2rad.(xref[3,:])
  xref[1:2,1] .-= [l1.x,l1.y]  
  xref = vcat(xref, zeros(1, size(xref,2)))
end
#%%


#%%
function init_model()
  model = Model(Ipopt.Optimizer)
  set_attribute(model, "print_level", 0)

  @variables model begin
    -Inf<=x<=Inf
    -Inf<=y<=Inf
    -pi<=θ<=pi
    -deg2rad(70)<=δ<=deg2rad(70)
    v
    ϕ
    ΔT[1:N] .== dT
  end

  states = [x, y, θ, δ]
  n_st = length(states)

  controls = [v,ϕ]
  n_ctrs = length(controls)

  @variables model begin
    U[i=1:n_ctrs,j=1:N]
    P[i=1:n_st*2]
    Uref[i=1:n_ctrs]
  end
  fix(Uref[1,1],v_ref)
  fix(Uref[2,1], 0.0)
  for i=1:2
    set_start_value(U[i,1],0.0)
  end

  set_upper_bound.(U[1,:],v_max)
  set_lower_bound.(U[1,:],v_min)
  set_upper_bound.(U[2,:],phi_max)
  set_lower_bound.(U[2,:],phi_min)

  f(st,con) = st + dT * [con[1]*cos(st[3]+st[4]),con[1]*sin(st[3]+st[4]),con[1]*sin(st[4])/L,con[2]]  
  # f(st,con) = st + dT * [con[1]*cos(st[3]),con[1]*sin(st[3]),con[1]*tan(st[4])/L,con[2]]  

  @expressions model begin                                                 
    X, zeros(NonlinearExpr, n_st, N+1)                                   
    X[:,1] .= P[1:4]                                                     
    for k = 1:N                                                          
      st = X[:,k]                                                      
      ctr = U[:,k]                                                     
      X[:,k+1] = f(st,ctr)                                             
    end                                                                  
  end

  Q = Diagonal([10.0,10.0,0.5,0.01])
  R = Diagonal([0.5,0.05])

  @expression(
    model, 
    obj, 
    permutedims(X[:,1:end-1] .- P[5:end]) * Q * (X[:,1:end-1] .- P[5:end]) + permutedims(U .- Uref) * R * (U .- Uref) |> diag |> sum)

  @objective(model, Min, obj)

  for k=1:N
    @constraint(model,X[:,k+1] .== f(X[:,k], U[:,k]))
  end
  # @constraint(model, U[1,N] == v_max)
  for i=1:n_st
    fix(P[i],x0[i])
  end
  return model
end
#%%

#%%
begin
  x0 = [xref[1:2,1]...,-xref[3,1], 0.0] #[(ego.get_location()+l1).x, (ego.get_location()+l1).y, ego.get_transform().rotation.yaw |> deg2rad,ego.get_control().steer |> deg2rad]
  model = init_model()
  print(model)
  set_ego_location(x0)
  #ego.set_transform(carla.Transform(ego.get_location(),carla.Rotation(yaw=180)))
  get_ego_view()
  xsol = [x0]
  #sim_time = 6
end
#%%
# p = plot(1,ls=:dash,
#   marker=:rect,
#   markersize=6,
#   xlim=(-300,10),
#   ylim=(-250,250),
#   label="trajectory",
#   legend=:outertop);
# #%% 
# #
# scatter!(p,xref[1,:],xref[2,:],
#   marker=:rect,
#   markersize=8,
#   c=:red,
#   label="target",
#   legend=:outertop,
#   alpha=0.25
# );
#%%

  

Uopt = Vector{Float64}[]
# anim = @animate
for x_ref in eachcol(xref[:,2:end])
  for i=1:length(model[:P])
   fix(model[:P][i],[x0...,x_ref...][i])
  end
  mpciter = 1
  st_ego = []
  #world.debug.draw_point(carla.Location(x=x_ref[1], y=x_ref[2]), size=0.25, life_time=7200, color=carla.Color(238, 18, 137, 0))
  while true #mpciter in 1:Int(sim_time/dT)
     optimize!(model)
     Uop = value.(model[:U])[:,1]
     global x0 = shift(value.(model[:X][:,1]),Uop)
     #control = ego.get_control()
     # control.throttle = 0.1
     #control.steer = x0[4]
     #ego.set_target_velocity(carla.Vector3D(Uop[1,1]*cos(x0[3]+x0[4]), Uop[1,1]*sin(x0[3]+x0[4]),0.0))
     #ego.apply_control(control)
     #get_ego_view()
     #tr = ego.get_transform()
     #ctrl = ego.get_control()
     #x0 = [(tr.location + l1).x, (tr.location + l1).y, deg2rad(tr.rotation.yaw),ctrl.steer]
     push!(xsol,x0)
     push!(Uopt,Uop)
     for i=1:2
       set_start_value(model[:U][i,1],Uop[i,1])
     end
     # if mpciter % 10 == 0
       # @info (mpciter,abs(x0[3] - x_ref[3]),norm(x0[1:2]-x_ref[1:2]))
     # end
     for i=1:4
       fix(model[:P][i],x0[i])
     end  
     if norm(x0[1:2]-x_ref[1:2]) < 0.1 #&& abs(x0[3] - x_ref[3])<deg2rad(3)
       break
     end
     mpciter += 1
  end
  @info (mpciter,abs(x0[3] - x_ref[3]),norm(x0[1:2]-x_ref[1:2])) 
end

#%%
xsol = mapreduce(permutedims,vcat,xsol)
Uopt = mapreduce(permutedims,vcat,Uopt)
#%%
#gif(anim;fps=5)



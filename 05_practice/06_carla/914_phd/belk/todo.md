# Set synchronous mode settings

new_settings = world.get_settings()
new_settings.synchronous_mode = True
new_settings.fixed_delta_seconds = 0.05
world.apply_settings(new_settings)

client.reload_world(False) # reload map keeping the world settings

# Set up the traffic manager

traffic_manager = client.get_trafficmanager(TM_PORT)
traffic_manager.set_synchronous_mode(True)
traffic_manager.set_random_device_seed(SEED) # define TM seed for determinism

1- logging to be added for later
2- use of tensor board
3- adding chart to view the paths, sensors etc, 2D window, might be using pygame

#./CarlaUE4.exe -windowed -ResX=800 -ResY=600 -carla-server -quality-level=Low

check also the ability to add an agent as a controller, this is our intelligent controller, how about multiple agents ?

# Testing Maths notation

---

$\mathbf{\text{Gradient Tree Boosting Algorithm}}$<br>

---

1.&emsp;Initialize model with a constant value $$f_{0}(x) = \textrm{arg min}_{\gamma} \sum \limits _{i=1} ^{N} L(y_{i}, \gamma)$$
2.&emsp;For m = 1 to M:<br>
&emsp;&emsp;(a)&emsp;For $i = 1,2,...,N$ compute<br>
$$r_{im} = - \displaystyle \Bigg[\frac{\partial L(y_{i}, f(x_{i}))}{\partial f(x_{i})}\Bigg]_{f=f_{m−1}}$$
&emsp;&emsp;(b)&emsp;Fit a regression tree to the targets $r_{im}$ giving terminal regions<br>
&emsp;&emsp;&emsp;&emsp;$R_{jm}, j = 1, 2, . . . , J_{m}.$<br><br>
&emsp;&emsp;(c)&emsp;For $j = 1, 2, . . . , J_{m}$ compute<br>
$$\gamma_{jm} = \underset{\gamma}{\textrm{arg min}} \sum \limits _{x_{i} \in R_{jm}} L(y_{i}, f_{m−1}(x_{i}) + \gamma)$$
&emsp;&emsp;(d)&emsp;Update $f_{m}(x) = f_{m−1}(x) + \sum _{j=1} ^{J_{m}} \gamma_{jm} I(x \in R_{jm})$<br><br>
3.&emsp;Output $\hat{f}(x) = f_{M}(x)$

---

Trying references to equations

```{math}
:label: my_label
w_{t+1} = (1 + r_{t+1}) s(w_t) + y_{t+1}
```

$$
  w_{t+1} = (1 + r_{t+1}) s(w_t) + y_{t+1}
$$ (my_other_label)

- A link to an equation directive: {eq}`my_label`
- A link to a dollar math block: {eq}`my_other_label`

Fixed

- [x] check list
- [ ] not working


Ref : https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/stanley_controller/stanley_controller.py

https://github.com/daniel-s-ingram/self_driving_cars_specialization/tree/master



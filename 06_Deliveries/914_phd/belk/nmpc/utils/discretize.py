def discretize_rk4(f_function, state_vector):
    # st_runge_kutta
    k1 = self.f(X[:,k], U[:,k])
    if self.activate_rk4:
        k2 = self.f(X[:,k]+self.dt/2*k1, U[:,k])
        k3 = self.f(X[:,k]+self.dt/2*k2, U[:,k])
        k4 = self.f(X[:,k]+self.dt/2*k3, U[:,k])
        st_rk4 = X[:,k]+self.dt/6*(k1+2*k2+2*k3+k4)
        st_next_aprx = st_rk4
    else:
        st_next_euler = X[:,k] + k1*self.dt
        st_next_aprx = st_next_euler
    st_next = X[:,k+1] 
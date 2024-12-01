from itertools import pairwise

class Spring:
    def __init__(self, k, offset) -> None:
        self.k = k
        self.offset = offset

    def iterate(self, sim, dt, f_in):
        return f_in - ((sim.x - self.offset) * self.k)

class Damper:
    def __init__(self, k) -> None:
        self.k = k

    def iterate(self, sim, dt, f_in):
        return f_in - (sim.v * self.k)

class ConstForce:
    def __init__(self, f) -> None:
        self.f = f

    def iterate(self, sim, dt, f_in):
        return f_in + self.f

class Friction:
    def __init__(self, f) -> None:
        self.f = f

    def iterate(self, sim, dt, f_in):
        if sim.v > 0.001:
            return f_in - self.f
        elif sim.v < 0.001:
            return f_in + self.f
        else:
            return f_in

class ForceMap:
    def __init__(self, x_vect, f_vect) -> None:
        self.x_vect = x_vect
        self.f_vect = f_vect

    def iterate(self, sim, dt, f_in):
        if sim.x <= self.x_vect[0]:
            return f_in - self.f_vect[0]
        elif sim.x >= self.x_vect[-1]:
            return f_in - self.f_vect[-1]
        else:
            for ((x0, x1), (f0, f1)) in zip(pairwise(self.x_vect), pairwise(self.f_vect)):
                if sim.x >= x0 and sim.x < x1:
                    k = (f1-f0) / (x1-x0)
                    d = f0 - (k * x0)
                    return f_in - ((k * sim.x) + d)
            return f_in - self.f_vect[-1]

class DampingMap:
    def __init__(self, x_vect, k_vect) -> None:
        self.x_vect = x_vect
        self.k_vect = k_vect

    def iterate(self, sim, dt, f_in):
        if sim.x <= self.x_vect[0]:
            return f_in - (sim.v * self.k_vect[0])
        elif sim.x >= self.x_vect[-1]:
            return f_in - (sim.v * self.k_vect[-1])
        else:
            for ((x0, x1), (k0, k1)) in zip(pairwise(self.x_vect), pairwise(self.k_vect)):
                if sim.x >= x0 and sim.x < x1:
                    k = (k1-k0) / (x1-x0)
                    d = k0 - (k * x0)
                    return f_in - (sim.v * ((k * sim.x) + d))
            return f_in - (sim.v * self.k_vect[-1])

class Sim:
    def __init__(self, m, x_min, x_max, v_min, v_max, a_min, a_max, f_deadzone) -> None:
        self.m = m
        self.x_min = x_min
        self.x_max = x_max
        self.v_min = v_min
        self.v_max = v_max
        self.a_min = a_min
        self.a_max = a_max
        self.f_deadzone = f_deadzone
        self.x = 0.0
        self.v = 0.0
        self.a = 0.0
        self.elements = []

    def iterate(self, dt, f_in):
        f_sum = f_in
        for element in self.elements:
            f_sum = element.iterate(self, dt, f_sum)
        if f_sum < self.f_deadzone and f_sum > -self.f_deadzone:
            f_sum = 0.0
        a = f_sum / self.m
        a_lim = max(min(a, self.a_max), self.a_min)
        v = self.v + (a_lim * dt)
        v_lim = max(min(v, self.v_max), self.v_min)
        x = self.x + (v_lim * dt)
        x_lim = max(min(x, self.x_max), self.x_min)
        if x != x_lim:
            v_lim = 0
        if v != v_lim:
            a_lim = 0
        self.a = a_lim
        self.v = v_lim
        self.x = x_lim

        return x_lim
    
if __name__ == '__main__':
    sim = Sim(0.5)
#    sim.elements.append(Spring(10.0, 1.0))
    sim.elements.append(ForceMap([-1.0, 3.0], [-20.0, 20.0]))
    sim.elements.append(Damper(1.0))
    sim.elements.append(ConstForce(2.0))
    sim.elements.append(ForceMap([-1.1, -1.0, 1.0, 1.1], [-2000.0, 0.0, 0.0, 2000.0]))
    sim.elements.append(DampingMap([-1.1, -1.0, -0.95, 0.95, 1.0, 1.1], [1000.0, 5.0, 0.0, 0.0, 5.0, 1000.0]))
    for i in range(100000):
        if i < 50000:
            f = -300
        else:
            f = 300
        x = sim.iterate(0.01, f)
        if i % 10000 == 0:
            print(x)


from bokeh.plotting import curdoc, figure
from bokeh.models import Div, Slider
from bokeh.layouts import column, row
from motion_sim import ConstForce, ForceMap, Damper, Sim, Spring, DampingMap, Friction
import time

sim = Sim(0.1, -1.2, 1.2, -10, 10, -100, 100, 0.5)
# sim.elements.append(Spring(10.0, 1.0))
sim.elements.append(ForceMap([-1.0, 3.0], [-20.0, 20.0]))
sim.elements.append(Damper(1.0))
sim.elements.append(ConstForce(2.0))
sim.elements.append(ForceMap([-1.1, -1.0, 1.0, 1.1], [-2000.0, 0.0, 0.0, 2000.0]))
sim.elements.append(DampingMap([-1.1, -1.0, -0.95, 0.95, 1.0, 1.1], [1000.0, 5.0, 0.0, 0.0, 5.0, 1000.0]))
sim.elements.append(Friction(1.0))

f_in = Slider(title="f_in", value=-2.2, start=-300.0, end=300.0, step=0.1)
m_in = Slider(title="f_in", value=0.1, start=0.01, end=10.0, step=0.01)

class BL:
    def __init__(self, x_graph, v_graph, a_graph, f_in, m_in) -> None:
        self.x_graph = x_graph
        self.v_graph = v_graph
        self.a_graph = a_graph
        self.f_in = f_in
        self.m_in = m_in
        self.t = 0.0
        self.f = f_in.value

    def update(self):
        t_vect = []
        x_vect = []
        v_vect = []
        a_vect = []
        for i in range(100):
            x_vect.append(sim.iterate(0.001, self.f))
            v_vect.append(sim.v)
            a_vect.append(sim.a)
            self.t += 0.01
            t_vect.append(self.t)

        self.x_graph.data_source.stream({'x': t_vect, 'y': x_vect})
        # self.v_graph.data_source.stream({'x': t_vect, 'y': v_vect})
        # self.a_graph.data_source.stream({'x': t_vect, 'y': a_vect})

    def update_data(self, attrname, old, new):
        # Get the current slider values
        self.f = self.f_in.value
        sim.m = self.m_in.value


p = figure()
x = p.line([], [])
v = p.line([], [])
a = p.line([], [])
bl = BL(x, v,  a, f_in, m_in)
f_in.on_change('value', bl.update_data)

curdoc().add_root(column(f_in, m_in, p, width=1500))

curdoc().add_periodic_callback(bl.update, 100)
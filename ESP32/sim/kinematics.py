import math
import numpy as np

def fit(x, y, max_order, max_err_ratio=0.01):
    num_samples = len(x)
    y_mean = np.mean(y)
    for order in range(0, max_order+1):
        (fit, meta) = np.polynomial.Polynomial.fit(x, y, order, full=True)
        err_mean_ratio = ((meta[0][0] / num_samples) ** 0.5) / y_mean
        if err_mean_ratio < max_err_ratio:
            print(f'sufficient fit @ order {order}: err_mean_ratio = {err_mean_ratio}')
            return fit.convert().coef
        
    print(f'best fit @ order {order}: err_mean_ratio = {err_mean_ratio}')
    return fit.convert().coef


l_pivot_foot = 180.0        # pedal length between pivot and foot contact point (mm)
l_pivot_link = 100.0        # pedal length between pivot and link (mm)
l_link = 149.0              # length of link between pedal and sled connection point (mm)
l_pivot_sled_y = 40.0       # vertical distance between pivot and sled connection point (mm)
l_pivot_sled_x_min = 104.0  # horizontal distance between pivot and sled connection point @ minimum stroke (mm)
l_sled_stroke = 100.0       # sled stroke (mm)

samples_per_mm = 10.0
max_order = 4
max_err = 0.005

data = {
    'l_sled': [],
    'r_force_link_foot': [],
    'l_foot': []
}

for i in range(int(l_sled_stroke * samples_per_mm) + 1):
    l_sled = float(i) / samples_per_mm
    l_pivot_sled_x = l_pivot_sled_x_min + l_sled
    phi_pivot_sled = math.atan2(l_pivot_sled_y, l_pivot_sled_x) # angle of the connecting line between pivot and sled connection point (from horizontal)
    l_pivot_sled = math.sqrt(l_pivot_sled_y ** 2 + l_pivot_sled_x ** 2) # length of the connecting line between pivot and sled connection point
    phi_link_pivot_sled = math.acos((l_link ** 2 - l_pivot_link ** 2 - l_pivot_sled ** 2) / (l_pivot_link * l_pivot_sled * -2)) # angle between the connecting line between pivot and link and the connecting line between pivot and sled connection point
    phi_link_sled_pivot = math.acos((l_pivot_link ** 2 - l_link ** 2 - l_pivot_sled ** 2) / (l_link * l_pivot_sled * -2)) # angle between the link and the connecting line between pivot and sled connection point
    phi_pivot_link_sled = math.pi - phi_link_pivot_sled - phi_link_sled_pivot # third angle of the triangle between pivot, link connection point and sled connection point
    phi_ped_vert = (math.pi / 2.0) - (phi_link_pivot_sled + phi_pivot_sled) # pedal angle (from vertical axis)
    phi_foot_link = (math.pi / 2.0) - phi_pivot_link_sled # angle between pedal normal and link
    r_force_link_foot = 1.0 / math.cos(phi_foot_link) * l_pivot_link / l_pivot_foot # ratio between load cell measurement (in link direction) and foot force (tangential to pedal)
    l_foot = phi_ped_vert * l_pivot_foot # foot position as an arc length relative to vertical
    data['l_sled'].append(l_sled)
    data['r_force_link_foot'].append(r_force_link_foot)
    data['l_foot'].append(l_foot)

print(f'fitting r_force_link_foot over l_sled...')
coeff_l_sled_to_r_force = fit(data['l_sled'], data['r_force_link_foot'], max_order, max_err)
print(f'coefficients: {str(coeff_l_sled_to_r_force).replace('.',',')}')

print()

print(f'fitting l_sled over l_foot...')
coeff_l_foot_to_l_sled = fit(data['l_foot'], data['l_sled'], max_order, max_err)
print(f'coefficients: {str(coeff_l_foot_to_l_sled).replace('.',',')}')


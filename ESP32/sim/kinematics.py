import math
import numpy as np

def fit_polynomial(x, y, max_order, max_err_ratio=0.01):
    num_samples = len(x)
    y_mean = np.mean(y)
    matching_order = None
    for order in range(0, max_order+1):
        (fit, meta) = np.polynomial.Polynomial.fit(x, y, order, full=True)
        err_mean_ratio = ((meta[0][0] / num_samples) ** 0.5) / y_mean
        if err_mean_ratio < max_err_ratio:
            print(f'sufficient fit @ order {order}: err_mean_ratio = {err_mean_ratio}')
            matching_order = order
            break
    if not matching_order:
        matching_order = order
        print(f'best fit @ order {order}: err_mean_ratio = {err_mean_ratio}')
    result = fit.convert().coef.tolist()
    result += [0.0] * (max_order - matching_order)
    return result


l_pivot_foot = 180.0        # pedal length between pivot and foot contact point (mm)
l_pivot_link = 100.0        # pedal length between pivot and link (mm)
l_link = 153.0              # length of link between pedal and sled connection point (mm)
l_pivot_sled_y = 32.0       # vertical distance between pivot and sled connection point (mm)
l_pivot_sled_x_min = 82.0   # horizontal distance between pivot and sled connection point @ minimum stroke (mm)
l_sled_stroke = 110.0       # nominal sled stroke (mm)

samples_per_mm = 10.0
max_order = 4
max_err = 0.001

data = {
    'l_sled': [],
    'r_force_link_foot': [],
    'l_foot': []
}

print(f'calculating kinematics @ {samples_per_mm} samples/mm ...')
for i in range(int(l_sled_stroke * samples_per_mm) + 1):
    l_sled = float(i) / samples_per_mm # sled position within stroke
    l_pivot_sled_x = l_pivot_sled_x_min + l_sled # horizontal position of the sled connection point relative to the pivot/origin
    phi_pivot_sled = math.atan2(l_pivot_sled_y, l_pivot_sled_x) # angle of the connecting line between pivot and sled connection point (from horizontal axis)
    l_pivot_sled = math.sqrt(l_pivot_sled_y ** 2 + l_pivot_sled_x ** 2) # length of the connecting line between pivot and sled connection point
    phi_link_pivot_sled = math.acos((l_link ** 2 - l_pivot_link ** 2 - l_pivot_sled ** 2) / (l_pivot_link * l_pivot_sled * -2)) # angle between the connecting line between pivot and link and the connecting line between pivot and sled connection point
    phi_link_sled_pivot = math.acos((l_pivot_link ** 2 - l_link ** 2 - l_pivot_sled ** 2) / (l_link * l_pivot_sled * -2)) # angle between the link and the connecting line between pivot and sled connection point
    phi_pivot_link_sled = math.pi - phi_link_pivot_sled - phi_link_sled_pivot # third angle of the triangle between pivot, link connection point and sled connection point
    phi_ped_vert = (math.pi / 2.0) - (phi_link_pivot_sled + phi_pivot_sled) # pedal angle (from vertical axis)
    phi_foot = -phi_ped_vert # foot angle (pedal normal, from horizontal axis)
    phi_link = -(math.pi / 2.0) - phi_ped_vert + phi_pivot_link_sled # link angle (from horizontal axis)
    phi_foot_link = phi_foot - phi_link # angle between foot force direction and link
    r_force_foot_link = (l_pivot_foot / l_pivot_link) * math.cos(phi_foot_link) # ratio between foot force and load cell measurement
    r_force_link_foot = 1.0 / r_force_foot_link # ratio between load cell measurement and foot force
    l_foot = phi_ped_vert * l_pivot_foot # foot position as an arc length relative to vertical axis
    data['l_sled'].append(l_sled)
    data['r_force_link_foot'].append(r_force_link_foot)
    data['l_foot'].append(l_foot)

x_foot_min = np.min(data['l_foot'])
x_foot_max = np.max(data['l_foot'])
print(f'foot travel limits: [{x_foot_min} {x_foot_max}]')

print(f'fitting r_force_link_foot over l_foot...')
coeff_l_foot_to_r_force = fit_polynomial(data['l_foot'], data['r_force_link_foot'], max_order, max_err)
print(f'coefficients: {str(coeff_l_foot_to_r_force)}')

print()

print(f'fitting l_sled over l_foot...')
coeff_l_foot_to_l_sled = fit_polynomial(data['l_foot'], data['l_sled'], max_order, max_err)
print(f'coefficients: {str(coeff_l_foot_to_l_sled)}')

TYPES = {
    'double': 'd',
    'float': 'f',
    'uint8_t': 'B',
    'int8_t': 'b',
    'uint16_t': 'H',
    'int16_t': 'h',
    'uint32_t': 'I',
    'int32_t': 'i',
    'uint64_t': 'Q',
    'int64_t': 'q',
}

import re
import struct

member_decl = re.compile(r'\s*(?P<type>\w+)\s+(?P<name>\w+)(\[(?P<size>\d+)\])?')

class StructMember:
    def __init__(self, fmt_code, name, size):
        self.fmt_code = fmt_code
        self.name = name
        self.size = int(size)
        self._value = [0] * self.size
    
    @property
    def value(self):
        return self._value
    
    @value.setter
    def value(self, value):
        if not isinstance(value, list):
            value = [value]
        if len(value) == len(self._value):
            self._value = value
        elif self.size > 1:
            raise ValueError(f'struct member "{self.name}" expects a list of {self.size} values')
        else:
            raise ValueError(f'struct member "{self.name}" expects exactly one value')
    
    @property
    def fmt_str(self):
        return f'{self.size}{self.fmt_code}'

class Struct:
    def __init__(self, name, definition):
        self.name = name
        self.members = []
        self.members_dict = {}
        for line in definition.split('\n'):
            line = line.strip()
            match = member_decl.match(line)
            if match:
                ctype = match['type']
                fmt_code = TYPES.get(ctype)
                if fmt_code:
                    size = match['size']
                    if not size:
                        size = 1
                    member = StructMember(fmt_code, match['name'], size)
                    self.members.append(member)
                    self.members_dict[member.name] = member
                else:
                    raise ValueError(f'Unknown type "{ctype}"!')
    
    def __setattr__(self, name, value):
        if name in ('name', 'members', 'members_dict'):
            self.__dict__[name] = value
            return
        self.members_dict[name].value = value

    def __getattr__(self, name):
        value = self.members_dict[name].value
        if len(value) == 1:
            return value[0]
        return value
    
    def set(self, name, value):
        self.members_dict[name].value = value
    
    def get(self, name):
        value = self.members_dict[name].value
        if len(value) == 1:
            return value[0]
        return value
    
    @property
    def value(self):
        values = []
        for member in self.members:
            values += member.value
        return values
    
    @property
    def fmt_str(self):
        fmt_str = '<'
        for member in self.members:
            fmt_str += member.fmt_str
        size = struct.calcsize(fmt_str)
        alligned_size = (size + 3) & 0xFFFC
        if size != alligned_size:
            fmt_str += f'{alligned_size - size}x'
        return fmt_str
 
    def to_bytes(self):
        return struct.pack(self.fmt_str, *self.value)
    
    def __len__(self):
        return struct.calcsize(self.fmt_str)

                
PAYLOAD_MECH = """
  // geometric properties of the pedal
  // in mm
  int16_t lengthPedal_a;
  int16_t lengthPedal_b;
  int16_t lengthPedal_d;
  int16_t lengthPedal_c_horizontal;
  int16_t lengthPedal_c_vertical;
  int16_t lengthPedal_travel;
  
  // polynomial coefficients for the conversion factor
  // from load cell force to pedal force over pedal position
  double coeffs_force_factor_over_pedal_pos[5];

  // polynomial coefficients for the conversion
  // from pedal position to sled position
  double coeffs_sled_pos_over_pedal_pos[5];

  // minimum absolute pedal position (0.1mm/LSB)
  int16_t x_foot_min_abs;

  // maximum absolute pedal position (0.1mm/LSB)
  int16_t x_foot_max_abs;

  // loadcell rating in kg / 2 --> to get value in kg, muiltiply by 2
  uint8_t loadcell_rating;

  // invert loadcell sign
  uint8_t invertLoadcellReading_u8;

  // invert motor direction
  uint8_t invertMotorDirection_u8;

  // spindle pitch in mm/rev
  uint8_t spindlePitch_mmPerRev_u8;

  //pedal type, 0= clutch, 1= brake, 2= gas
  uint8_t pedal_type;
"""

mech_struct = Struct('mech', PAYLOAD_MECH)
mech_struct.coeffs_force_factor_over_pedal_pos = coeff_l_foot_to_r_force
mech_struct.coeffs_sled_pos_over_pedal_pos = coeff_l_foot_to_l_sled
mech_struct.x_foot_min_abs = int(x_foot_min * 10.0)
mech_struct.x_foot_max_abs = int(x_foot_max * 10.0)
mech_struct.loadcell_rating = 100
mech_struct.invertLoadcellReading_u8 = 0
mech_struct.invertMotorDirection_u8 = 0
mech_struct.spindlePitch_mmPerRev_u8 = 5
mech_struct.pedal_type = 1

print(f'mech_struct is {len(mech_struct)} bytes long')
print(mech_struct.to_bytes().hex(' '))

PAYLOAD_GENERAL = """
  // configure pedal start and endpoint
  // In percent
  uint8_t pedalStartPosition;
  uint8_t pedalEndPosition;

  // configure pedal forces
  float maxForce;
  float preloadForce;
  
  // design force vs travel curve
  // In percent
  uint8_t relativeForce_p000; 
  uint8_t relativeForce_p020;
  uint8_t relativeForce_p040;
  uint8_t relativeForce_p060;
  uint8_t relativeForce_p080;
  uint8_t relativeForce_p100;

  // parameter to configure damping
  float dampingPress;
  float dampingPull;

  // configure ABS effect 
  uint8_t absFrequency; // In Hz
  uint8_t absAmplitude; // In kg/20
  uint8_t absPattern; // 0: sinewave, 1: sawtooth
  uint8_t absForceOrTarvelBit; // 0: Force, 1: travel

  //Simulate ABS trigger
  uint8_t Simulate_ABS_trigger;
  uint8_t Simulate_ABS_value;
  // configure for RPM effect
  uint8_t RPM_max_freq; //In HZ
  uint8_t RPM_min_freq; //In HZ
  uint8_t RPM_AMP; //In Kg

  //configure for bite point
  uint8_t BP_trigger_value;
  uint8_t BP_amp;
  uint8_t BP_freq;
  uint8_t BP_trigger;
  //G force effect
  uint8_t G_multi;
  uint8_t G_window;
  //wheel slip
  uint8_t WS_amp;
  uint8_t WS_freq;
  //Road impact effect
  uint8_t Road_multi;
  uint8_t Road_window;
  //Custom Vibration 1
  uint8_t CV_amp_1;
  uint8_t CV_freq_1;
  //Custom Vibration 2
  uint8_t CV_amp_2;
  uint8_t CV_freq_2;
  // cubic spline parameters
  float cubic_spline_param_a_array[5];
  float cubic_spline_param_b_array[5];

  // Kalman filter model noise
  uint8_t kf_modelNoise;
  uint8_t kf_modelOrder;

  // debug flags, used to enable debug output
  uint8_t debug_flags_0;

  // use loadcell or travel as joystick output
  uint8_t travelAsJoystickOutput_u8;
"""
general_struct = Struct('gneral', PAYLOAD_GENERAL)

print(f'general_struct is {len(general_struct)} bytes long')
print(general_struct.to_bytes().hex(' '))


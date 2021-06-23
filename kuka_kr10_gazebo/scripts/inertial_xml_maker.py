from __future__ import print_function


def print_xml(i, s, m, v, com, d):
    v = v / s**3
    com = [x / s for x in com]

    for key in d.keys():
        d[key] = d[key] / s**5 * m / v

    print("link_{}".format(i))
    print("<inertial>")
    print("  <mass value=\"{}\" />".format(m))
    print("  <origin xyz=\"{} {} {}\" rpy=\"0 0 0\" />".format(com[0], com[1], com[2]))
    print("  <inertia ixx=\"{ixx}\" ixy=\"{ixy}\" ixz=\"{ixz}\" iyy=\"{iyy}\" iyz=\"{iyz}\" izz=\"{izz}\" />".format(**d))
    print("</inertial>")
    print()

    return


if __name__ == '__main__':
    s = 10

    # base_link
    m = 14
    v = 10.497350
    com = (-0.444859, -0.000497, 1.100921)
    d = {'ixx': 6.656230, 'ixy': -0.009825, 'ixz': -0.112668, 'iyy': 10.717628, 'iyz': -0.001689, 'izz': 9.467064}
    print_xml(0, s, m, v, com, d)

    # link_1
    m = 11.2
    v = 11.404981
    com = (0.146723, 0.000546, -0.557645)
    d = {'ixx': 10.997753, 'ixy': 0.026161, 'ixz': -0.457176, 'iyy': 9.092752, 'iyz': -0.018954, 'izz': 8.535112}
    print_xml(1, s, m, v, com, d)

    # link_2
    m = 8.4
    v = 10.556457
    com = (2.482357, -0.029221, 0.079418)
    d = {'ixx': 4.590066, 'ixy': 0.281169, 'ixz': -0.222809, 'iyy': 47.571838, 'iyz': 0.000364, 'izz': 49.331104}
    print_xml(2, s, m, v, com, d)

    # link_3
    m = 8.4
    v = 2.869388
    com = (0.160924, 0.000238, 0.105013)
    d = {'ixx': 0.815426, 'ixy': 0.000059, 'ixz': -0.139995, 'iyy': 1.110488, 'iyz': -0.000381, 'izz': 0.981673}
    print_xml(3, s, m, v, com, d)

    # link_4
    m = 5.6
    v = 4.029067
    com = (3.011480, -0.004543, 0.002350)
    d = {'ixx': 0.730050, 'ixy': 0.008849, 'ixz': -0.006082 , 'iyy': 5.072010, 'iyz': 0.000064, 'izz': 5.163661}
    print_xml(4, s, m, v, com, d)

    # link_5
    m = 5.6
    v = 0.441257
    com = (0.157906, 0.019162, -0.002449)
    d = {'ixx': 0.031747, 'ixy': 0.001659, 'ixz': 0.000363, 'iyy': 0.052697, 'iyz': 0.000010, 'izz': 0.056085}
    print_xml(5, s, m, v, com, d)

    # link_6
    m = 2.8
    v = 0.013976
    com = (-0.075312, 0.000001, -0.009341)
    d = {'ixx': 0.000301, 'ixy': -0.000000, 'ixz': -0.000002, 'iyy': -0.000002, 'iyz': -0.000000, 'izz': 0.000170}
    print_xml(6, s, m, v, com, d)


import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hexmovr02/hexmovr_manager/install/hexmovr_moto_manager'

def is_number_float(value):
    try:
        float(value)
        return True
    except ValueError:
        return False
    
def is_number_int(value):
    try:
        int(value)
        return True
    except ValueError:
        return False
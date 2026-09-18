import time
import math

# 记录开始时间
start_time = time.time()

# 你的原始代码
m = 130
n = 316
R = 74.6920
r = 34.6410

x = 0
y = 0
z = -237.5

A1 = 2 * (r - R + x) * m
B1 = -2 * z * m
C1 = (r - R + x)*(r - R + x) + y*y + z*z + m*m - n*n

A2 = 2 * (r - R - 0.5 * x + 0.8660 * y) * m
B2 = -2 * z * m
C2 = (r - R - 0.5 * x + 0.8660 * y)*(r - R - 0.5 * x + 0.8660 * y) + (0.8660 * x + 0.5 * y)*(0.8660 * x + 0.5 * y) + z*z + m*m - n*n

A3 = 2 * (r - R - 0.5 * x - 0.8660 * y) * m
B3 = -2 * z * m
C3 = (r - R - 0.5 * x - 0.8660 * y)*(r - R - 0.5 * x - 0.8660 * y) + (0.8660 * x - 0.5 * y)*(0.8660 * x - 0.5 * y) + z*z + m*m - n*n

def calculate_angle(A, B, C):
    print(A * 65536, B * 65536, C * 65536)
    
    discriminant = A*4096*A*4096 + B*4096*B*4096 - C*4096*C*4096
    
    if discriminant < 0:
        raise ValueError("Discriminant is negative, resulting in a complex number.")
    
    sqrt_discriminant = math.sqrt(discriminant)
    
    print(sqrt_discriminant)
    
    Numerator = -A*4096*B*4096 - C * 4096 * sqrt_discriminant
    Denominator = A*4096*A*4096 - C*4096*C*4096
    
    if Denominator == 0:
        raise ValueError("Denominator is zero, leading to division by zero error.")
    
    Quotient = Numerator / Denominator
    
    print(Numerator)
    print(Denominator)   
    print( ) 
    print( ) 
    print( ) 
      
    if Quotient < 0:
        result = math.atan(-Quotient)
        angle = 180 - math.degrees(result)
    else:   
        result = math.atan(Quotient)
        angle = math.degrees(result)
    
    return angle

angle1 = calculate_angle(A1, B1, C1)
angle2 = calculate_angle(A2, B2, C2)
angle3 = calculate_angle(A3, B3, C3)

print((130 - angle1) * 256)
print((130 - angle2) * 256)
print((130 - angle3) * 256)

# 记录结束时间
end_time = time.time()

# 计算并打印运行时间
execution_time = end_time - start_time
print(f"代码运行时间: {execution_time:.6f} 秒")
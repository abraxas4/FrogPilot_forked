def clip(x, lo, hi):
  # Ensures that 'x' is within the range ['lo', 'hi'].
  # If 'x' is less than 'lo', 'lo' is returned.
  # If 'x' is greater than 'hi', 'hi' is returned.
  # Otherwise, 'x' is returned unchanged.
  return max(lo, min(hi, x))

def interp(x, xp, fp):
  # Perform linear interpolation.
  # 'x' is the x-coordinate at which to evaluate the interpolation.
  # 'xp' is an array of x-coordinates where 'fp', the function values, are known.
  # 'fp' is an array of y-coordinates known at 'xp'.
  # The function will return the interpolated value at 'x'.
  
  N = len(xp)

  def get_interp(xv):
    # Find index 'hi' such that 'xv' is between 'xp[hi-1]' and 'xp[hi]'.
    hi = 0
    while hi < N and xv > xp[hi]:
      hi += 1
    low = hi - 1
    # If 'xv' is beyond the range of 'xp', return the edge values.
    # Otherwise, compute the interpolated value using the formula for linear interpolation.
    return fp[-1] if hi == N and xv > xp[low] else (
      fp[0] if hi == 0 else
      (xv - xp[low]) * (fp[hi] - fp[low]) / (xp[hi] - xp[low]) + fp[low])

  # If 'x' is iterable, apply interpolation to each element and return a list.
  # If 'x' is a single value, just interpolate it.
  return [get_interp(v) for v in x] if hasattr(x, '__iter__') else get_interp(x)
'''
example:

x = 2.5
xp = [1, 2, 3]
fp = [3, 6, 9]
interpolated_value = interp(x, xp, fp)
print(interpolated_value)  # Output should be 7.5, since it's halfway between 6 and 9

x = [1.5, 2.5, 3.5]
xp = [1, 2, 3, 4]
fp = [3, 6, 9, 12]
interpolated_values = interp(x, xp, fp)
print(interpolated_values)  # Output should be [4.5, 7.5, 10.5]

x = 0.5  # Value is less than the smallest xp
xp = [1, 2, 3]
fp = [3, 6, 9]
interpolated_value = interp(x, xp, fp)
print(interpolated_value)  # Output should be 3

x = 4  # Value is greater than the largest xp
interpolated_value = interp(x, xp, fp)
print(interpolated_value)  # Output should be 9
#endif
'''

def mean(x):
  # Calculate the arithmetic mean (average) of a list 'x'.
  # This is the sum of all elements in 'x', divided by the number of elements.
  return sum(x) / len(x)

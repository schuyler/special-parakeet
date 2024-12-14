@lazyglobal off.

// Minimize a function value
function minimize {
  // this is basically ternary search straight off Wikipedia
  parameter func, a, b.
  parameter epsilon is 0.2.
  parameter nmax is 1000.

  local n is 0.
  local m1 is 0.
  local m2 is 0.
  until n > nmax or abs(b - a) < epsilon {
    //print "A: " + round(a, 1) + " F(a): " + round(func(a), 1) + " B: " + round(b, 1) + " F(b): " + round(func(b),1).
    set m1 to a + (b - a) / 3.
    set m2 to b - (b - a) / 3.
    if func(m1) > func(m2) {
	    set a to m1.
    } else {
	    set b to m2.
    }
    set n to n + 1.
  }
  return (a + b) / 2.
}

function test_minimize {
  function f {
    parameter x.
    return (x - 3)^2.
  }
  local f_min is minimize(f@, 0, 10, 0.001).
  print "Expected: 3.0".
  print "Estimated: " + f_min.
}

// test_minimize.
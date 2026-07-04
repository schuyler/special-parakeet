@lazyGlobal off.

// === Optimization functions ===

function find_zero { // of a function using the Newton-Raphson method
    parameter f.
    parameter df. // Derivative of f.
    parameter x0.
    parameter epsilon is 0.0001.
    parameter max_iterations is 100.
    parameter debug is false.

    local x is x0.
    local deltaX is 1.0.
    local iteration is 0.

    // Newton-Raphson finds roots of f(x) = 0 using: x_{n+1} = x_n - f(x_n)/f'(x_n)
    // It stops when the change in x is less than epsilon or after max_iterations.

    until abs(deltaX) < epsilon or iteration > max_iterations {
        set iteration to iteration + 1.
        local df_x to df(x).
        if df_x = 0 { // Avoid division by zero
            print "Derivative is zero at x = " + round(x, 6) + ". Stopping iteration.".
            return x.
        }
        set deltaX to f(x) / df_x.
        set x to x - deltaX.
        if debug {
            print "Iteration " + iteration + ": x = " + x + ", f(x) = " + f(x) + ", df(x) = " + df(x) + ".".
        }
    }
    
    //print "Iterations: " + iteration + ", final x = " + x + ", f(x) = " + f(x) +".".
    return x.
}

function bisect {
    parameter f.
    parameter start.
    parameter end.
    parameter epsilon is 0.0001.
    parameter max_iterations is 100.
    parameter debug is false.

    local a is start.
    local b is end.
    local c is (a + b) / 2.
    local iteration is 0.
    // Bisection method finds roots of f(x) = 0 by repeatedly halving the interval [a, b] where f(a) and f(b) have opposite signs.
    if f(a) * f(b) > 0 {
        print "Bisection bracketing failed:".
        print "  f(" + round(a,2) + ") = " + round(f(a),4).
        print "  f(" + round(b,2) + ") = " + round(f(b),4).
        print "  Both have same sign - no root in interval".
        return -1.
    }

    local f_a is f(a).
    local f_c is f(c).
    until abs(b - a) < epsilon or iteration > max_iterations {
        set iteration to iteration + 1.
        if f_c = 0 {
            return c. // Found exact root
        }
        if f_a * f_c < 0 {
            set b to c. // Root is in [a, c]
        } else {
            set a to c. // Root is in [c, b]
            set f_a to f_c. // Update f(a) to the new value
        }
        set c to (a + b) / 2.
        set f_c to f(c).
        if debug {
            // Print the current state of the bisection method.
            print "Iteration " + iteration + ": a = " + a + ", b = " + b + ", c = " + c + ", f(c) = " + f_c + ".".
        }
    }
    return c.
}

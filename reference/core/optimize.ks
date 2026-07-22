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

// Ternary search for the minimum of a unimodal function on [a, b].
function minimize {
    parameter func, a, b.
    parameter epsilon is 0.2.
    parameter nmax is 1000.

    local n is 0.
    local m1 is 0.
    local m2 is 0.
    until n > nmax or abs(b - a) < epsilon {
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

// Minimize func over [a, b] without assuming it's unimodal there, which
// bare ternary search does: coarse-scan the interval, bracket the best
// sample, then hand the bracket to minimize. Guards against a minimum
// sitting on (or just past) a boundary and against multiple local dips.
function minimize_scan {
    parameter func, a, b.
    parameter epsilon is 0.2.
    parameter samples is 24.

    local step is (b - a) / samples.
    local best_i is 0.
    local best_f is func(a).
    local i is 1.
    until i > samples {
        local f is func(a + i * step).
        if f < best_f {
            set best_f to f.
            set best_i to i.
        }
        set i to i + 1.
    }
    local lo is a + max(0, best_i - 1) * step.
    local hi is a + min(samples, best_i + 1) * step.
    return minimize(func, lo, hi, epsilon).
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

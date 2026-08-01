// columns.ks -- render a list of values as a fixed-width row for the console.
//
// A logging script renders one list of values two ways, so that the console a
// pilot watches and the CSV a flight is judged by cannot drift apart. The CSV
// takes the list through the list's own JOIN; the console takes it through
// here.
//
// Widths hold the widest value each column can produce. A value that outgrows
// its width keeps every digit and loses only its leading space, so a row stays
// readable and a number is never quietly shortened.

@lazyglobal off.

function columns {
  parameter values, widths.
  local line is "".
  local i is 0.
  until i >= values:length {
    local text is "" + values[i].
    set line to line + text:padleft(widths[i]).
    set i to i + 1.
  }
  return line.
}

// Select a subset of a list's elements by index, so a printed row can leave
// columns out without ever disagreeing with the CSV row it came from --
// same values, same names, same widths, fewer of them. It lives here rather
// than in a caller because it is the second half of the one-row-two-ways
// idea above: a console narrower than the full row is the normal case, not
// one script's problem.
function subset {
  parameter lst, idx.
  local out is list().
  local i is 0.
  until i >= idx:length {
    out:add(lst[idx[i]]).
    set i to i + 1.
  }
  return out.
}

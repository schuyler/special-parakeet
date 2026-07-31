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

# boot_filter.awk: the quiet terminal of wifi_3d_nav.sh (not with --verbose).
#
#   awk -v LOGFILE=boot.log -f boot_filter.awk
#   (mawk: add -W interactive, or lines wait in its input buffer)
#
# Every line goes to LOGFILE. The terminal gets only what needs a person:
#   * [BOOT] and [PROGRESS n/N] progress lines
#   * warnings and errors (WARN, ERROR, FAIL, TOO LARGE, MISSING, ...) with
#     their indented follow-up lines (the "Fix:" / "Install:" hints)
#   * [link] messages, password prompts, and whole PROBLEM boxes
#   * the Ctrl+C shutdown steps (what is being saved, how to skip the copy)
# ROS launch output ([node-1] ..., [WARN] [stamp] [node]: ...) is shown only
# for ERROR / FATAL. A line seen twice is not repeated (digits ignored), and a
# note says where the rest is.
# -v TTY=1 (the terminal is a tty): each printed line first clears the line, where
# wifi_3d_nav.sh may have drawn its progress spinner (it writes past this filter).
# Plain POSIX awk (Ubuntu's mawk): no strftime, no {n} intervals.

function clean(s) {
    gsub(/\033\[[0-9;]*[A-Za-z]/, "", s)
    return s
}

function first(s,    k) {
    k = s
    gsub(/[0-9]+(\.[0-9]+)?/, "#", k)
    seen[k]++
    if (seen[k] <= 2) return 1
    if (!noted) {
        if (TTY == 1) printf "\r\033[K"
        print "         (repeated messages are hidden; everything is in " LOGFILE ")"
        noted = 1
    }
    return 0
}

{
    raw = $0
    print raw > LOGFILE
    fflush(LOGFILE)
    c = clean(raw)
    show = 0
    if (box) {
        show = 1
        if (c ~ /^=+[ ]*$/) box = 0
    } else if (c ~ /PROBLEM:/) {
        if (TTY == 1) printf "\r\033[K"
        print "=================================================================="
        box = 1; show = 1
    } else if (c ~ /^\[BOOT\]/ || c ~ /^\[PROGRESS / || c ~ /^Stopping safely|^Stopping SLAM on the Jetson|^Copying the 3D map|^[ ]+python3 .*fetch-lidar3d|^Transfer (skipped|incomplete)|^\[wifi_nav\] (Shutting down|Done)/) {
        show = 1
    } else if (c ~ /^[ ]*\[[A-Za-z0-9_.\/-]+-[0-9]+\]/ || c ~ /^\[(INFO|WARN|ERROR|FATAL|DEBUG)\]/ || c ~ /^[0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9]+\.[0-9]+ \[/) {
        if (c ~ /ERROR|FATAL/) show = first(c)
    } else if (cont && c ~ /^[ ][ ][ ][ ]+[^ ]/) {
        show = 1
    } else if (c ~ /WARN|ERROR|FAIL|TOO LARGE|MISSING|NOT SET UP|Traceback|^\[link\]|[Pp]assword|^Connecting to /) {
        show = first(c)
    }
    cont = show && !box
    if (show) {
        if (TTY == 1) printf "\r\033[K"
        print raw
        fflush()
    }
}

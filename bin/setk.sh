# Copy of crisp_lab/bin/setk.sh for the container.  Use: ./fr3 setk [mode] <k_pos> <k_rot>      e.g.  ./fr3 setk real 600 20
# Sets task.k_pos_{x,y,z} and task.k_rot_{x,y,z} of the running cartesian_impedance_controller live (no restart).
# Whole numbers are fine (converted to doubles, which ros2 param set insists on). Rejects non-numbers and values
# above 3000 N/m / 300 Nm/rad without touching anything. Lost on a stack restart: write final values into controllers_real.yaml.
setk() {
  local kp kr a re='^[0-9]+([.][0-9]+)?$'
  [[ $1 =~ $re && $2 =~ $re ]] || { echo "usage: setk <k_pos> <k_rot>   (positive numbers); nothing was changed"; return 1; }
  kp=$(printf '%.1f' "$1") kr=$(printf '%.1f' "$2")
  awk -v p="$kp" -v r="$kr" 'BEGIN{exit !(p>3000 || r>300)}' && { echo "refusing: above 3000 N/m or 300 Nm/rad; nothing was changed"; return 1; }
  for a in x y z; do
    ros2 param set /cartesian_impedance_controller task.k_pos_$a "$kp" 2>&1 | grep -q successful || { echo "FAILED to set k_pos_$a (stopping)"; return 1; }
  done
  for a in x y z; do
    ros2 param set /cartesian_impedance_controller task.k_rot_$a "$kr" 2>&1 | grep -q successful || { echo "FAILED to set k_rot_$a (stopping)"; return 1; }
  done
  echo "gains set: k_pos $kp  k_rot $kr"
}

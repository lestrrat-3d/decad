package tessellation

// ZipColumns triangulates the strip between two adjacent vertex columns
// of one wall, a and b ascending at heights za and zb from one common bottom
// to one common top: it climbs whichever column's next vertex is lower, b
// first at a shared height, so two columns holding the same heights close
// in quads split as (a, b, b↑) and (a, b↑, a↑).
func ZipColumns(a, b []int, za, zb []float64, emit func([3]int)) {
	i, k := 0, 0
	for i < len(a)-1 || k < len(b)-1 {
		if i == len(a)-1 || (k < len(b)-1 && zb[k+1] <= za[i+1]) {
			emit([3]int{a[i], b[k], b[k+1]})
			k++
			continue
		}
		emit([3]int{a[i], b[k], a[i+1]})
		i++
	}
}

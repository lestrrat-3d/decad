package examples_test

import (
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad"
)

// igesTetrahedron is a tetrahedron with legs of 6 mm as an IGES file: a vertex list, six lines, an edge list,
// four planes with their loops and faces, a shell and the manifold solid B-rep that owns it. Each entity is two
// directory records and its parameter records, in the fixed 80-column layout.
func igesTetrahedron() string {
	var dir, data []string
	add := func(typ int, params ...string) int {
		seq := len(dir) + 1
		text := fmt.Sprintf("%d,%s;", typ, strings.Join(params, ","))
		first := len(data) + 1
		count := 0
		for len(text) > 0 {
			n := min(64, len(text))
			data = append(data, fmt.Sprintf("%-64s%8dP%7d", text[:n], seq, len(data)+1))
			text = text[n:]
			count++
		}
		dir = append(dir,
			fmt.Sprintf("%8d%8d%8d%8d%8d%8d%8d%8d00000000D%7d", typ, first, 0, 0, 0, 0, 0, 0, seq),
			fmt.Sprintf("%8d%8d%8d%8d%8d%32sD%7d", typ, 0, 0, count, 0, "", seq+1))
		return seq
	}
	corners := [][3]string{{"0", "0", "0"}, {"6", "0", "0"}, {"0", "6", "0"}, {"0", "0", "6"}}
	list := []string{"4"}
	for _, c := range corners {
		list = append(list, c[0], c[1], c[2])
	}
	vertices := add(502, list...)
	ends := [][2]int{{1, 2}, {1, 3}, {1, 4}, {2, 3}, {2, 4}, {3, 4}}
	edges := []string{"6"}
	for _, e := range ends {
		a, b := corners[e[0]-1], corners[e[1]-1]
		line := add(110, a[0], a[1], a[2], b[0], b[1], b[2])
		edges = append(edges, fmt.Sprint(line), fmt.Sprint(vertices), fmt.Sprint(e[0]), fmt.Sprint(vertices), fmt.Sprint(e[1]))
	}
	edgeList := add(504, edges...)
	// Each face: its plane A, B, C, D and the edges of its loop with their sense (1 along the edge, 0 against it).
	faces := []struct {
		plane string
		loop  [][2]int
	}{
		{"0,0,1,0", [][2]int{{1, 1}, {4, 1}, {2, 0}}},
		{"0,1,0,0", [][2]int{{1, 1}, {5, 1}, {3, 0}}},
		{"1,0,0,0", [][2]int{{2, 1}, {6, 1}, {3, 0}}},
		{"1,1,1,6", [][2]int{{4, 1}, {6, 1}, {5, 0}}},
	}
	shell := []string{"4"}
	for _, f := range faces {
		plane := add(108, f.plane, "0")
		loop := []string{fmt.Sprint(len(f.loop))}
		for _, u := range f.loop {
			loop = append(loop, "0", fmt.Sprint(edgeList), fmt.Sprint(u[0]), fmt.Sprint(u[1]), "0")
		}
		lp := add(508, loop...)
		face := add(510, fmt.Sprint(plane), "1", "1", fmt.Sprint(lp))
		shell = append(shell, fmt.Sprint(face), "1")
	}
	sh := add(514, shell...)
	add(186, fmt.Sprint(sh), "1", "0")
	global := "1H,,1H;,4HTEST,8Htet.iges,5Hdecad,3H1.0,32,38,6,308,15,4HTEST,1.0,2,2HMM,1,0.01;"
	out := []string{fmt.Sprintf("%-72sS%7d", "a tetrahedron", 1)}
	records := 0
	for len(global) > 0 { // the Global section runs on over records of 72 columns
		n := min(72, len(global))
		records++
		out = append(out, fmt.Sprintf("%-72sG%7d", global[:n], records))
		global = global[n:]
	}
	out = append(out, dir...)
	out = append(out, data...)
	out = append(out, fmt.Sprintf("S%7dG%7dD%7dP%7d%40sT%7d", 1, records, len(dir), len(data), "", 1))
	return strings.Join(out, "\n") + "\n"
}

// ReadIGES reads the planar-faced solids of an IGES file into a mesh body, as ReadSTEP does for
// STEP: each face on a plane is cut into triangles, the faces are wound outward, and lengths are
// converted from the file's unit to millimetres. SolidFromMesh then reads the mesh as a solid. A
// face on a curved surface, an arc edge or a placed entity is refused, not guessed at.
func Example_read_iges() {
	mesh, err := decad.ReadIGES(context.Background(), strings.NewReader(igesTetrahedron()))
	if err != nil {
		fmt.Printf("failed to read the file: %s\n", err)
		return
	}
	solid, err := decad.SolidFromMesh(context.Background(), decad.New(), mesh)
	if err != nil {
		fmt.Printf("failed to make a solid: %s\n", err)
		return
	}
	volume, err := solid.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("faces: %d, volume: %.0f mm^3\n", len(solid.Faces()), volume.Value.Base())
	// Output:
	// faces: 4, volume: 36 mm^3
}

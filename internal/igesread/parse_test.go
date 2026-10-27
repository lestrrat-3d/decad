package igesread_test

import (
	"fmt"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad/internal/igesread"
	"github.com/stretchr/testify/require"
)

// file builds an IGES file with the global text and one entity of the type with the parameter text.
func file(global string, pdata string) string {
	rec := func(text string, section byte, seq int) string {
		return fmt.Sprintf("%-72s%c%7d", text, section, seq)
	}
	f := func(v int) string { return fmt.Sprintf("%8d", v) }
	var out []string
	out = append(out, rec("test", 'S', 1))
	for i, g := range chunks(global, 72) {
		out = append(out, rec(g, 'G', i+1))
	}
	out = append(out, f(110)+f(1)+f(0)+f(0)+f(0)+f(0)+f(0)+f(0)+"00000000"+"D      1")
	out = append(out, f(110)+f(0)+f(0)+f(1)+f(0)+strings.Repeat(" ", 32)+"D      2")
	out = append(out, fmt.Sprintf("%-64s%8d%c%7d", pdata, 1, 'P', 1))
	out = append(out, fmt.Sprintf("S%7dG%7dD%7dP%7d%sT%7d", 1, len(chunks(global, 72)), 2, 1, strings.Repeat(" ", 40), 1))
	return strings.Join(out, "\n") + "\n"
}

func chunks(s string, n int) []string {
	var out []string
	for len(s) > 0 {
		k := min(n, len(s))
		out = append(out, s[:k])
		s = s[k:]
	}
	return out
}

// global builds a global section with the delimiters and the unit flag, the model space scale (13) and the unit (14, 15)
// at their places.
func global(pd, rd string, flag int) string {
	fields := []string{"4HTEST", "8Hbox.iges", "5Hdecad", "3H1.0", "32", "38", "6", "308", "15", "4HTEST", "1.0",
		fmt.Sprint(flag), "2HMM", "1", "0.01"}
	return "1H" + pd + "," + "1H" + rd + "," + strings.Join(fields, pd) + rd
}

func TestParseReadsDelimitersUnitsAndParameters(t *testing.T) {
	t.Parallel()
	line := "110,0.,0.,0.,1.5D+01,2.5,-3.;"
	for name, tc := range map[string]struct {
		global string
		scale  float64
		params string
	}{
		"spelled out":       {global(",", ";", 2), 1, line},
		"inches":            {global(",", ";", 1), 25.4, line},
		"custom delimiters": {global("/", "|", 6), 1000, "110/0./0./0./1.5D+01/2.5/-3.|"},
		"centimetres":       {global(",", ";", 10), 10, line},
		"feet":              {global(",", ";", 4), 304.8, line},
	} {
		m, err := igesread.Parse([]byte(file(tc.global, tc.params)))
		require.NoError(t, err, name)
		require.Equal(t, tc.scale, m.Scale, name)
		require.InDelta(t, 1, m.ModelScale, 0, name)
		e := m.Entities[1]
		require.NotNil(t, e, name)
		require.Equal(t, 110, e.Type)
		require.Equal(t, []string{"0.", "0.", "0.", "1.5D+01", "2.5", "-3."}, e.Params, name)
		xs, err := igesread.Reals(e.Params)
		require.NoError(t, err)
		require.Equal(t, []float64{0, 0, 0, 15, 2.5, -3}, xs, "the D exponent reads as E")
	}
	// A Hollerith string holds a delimiter without ending its parameter.
	text := file(global(",", ";", 2), "406,0.,5Ha,b;c,3;")
	m, err := igesread.Parse([]byte(strings.Replace(text, "     110", "     406", 2)))
	require.NoError(t, err)
	require.Equal(t, []string{"0.", "a,b;c", "3"}, m.Entities[1].Params)
}

func TestParseRefusesWhatIsNotWellFormed(t *testing.T) {
	t.Parallel()
	good := file(global(",", ";", 2), "110,0.,0.,0.,1.,2.,3.;")
	_, err := igesread.Parse([]byte(good))
	require.NoError(t, err)
	// CRLF line ends and short lines are read.
	crlf := strings.ReplaceAll(good, "\n", "\r\n")
	_, err = igesread.Parse([]byte(crlf))
	require.NoError(t, err)
	for name, text := range map[string]string{
		"no terminate":         strings.Join(strings.Split(good, "\n")[:4], "\n") + "\n",
		"unknown section":      strings.Replace(good, "G      1", "X      1", 1),
		"type disagrees":       strings.Replace(good, "110,0.,", "111,0.,", 1),
		"an unknown unit flag": file(global(",", ";", 99), "110,0.,0.,0.,1.,2.,3.;"),
		"no record delimiter":  file(global(",", ";", 2), "110,0.,0.,0.,1.,2.,3."),
		"a bad real":           file(global(",", ";", 2), "110,0.,0.,0.,1.,2.,x;"),
	} {
		m, err := igesread.Parse([]byte(text))
		if err == nil && name == "a bad real" {
			_, err = igesread.Reals(m.Entities[1].Params)
		}
		require.ErrorIs(t, err, igesread.ErrMalformed, name)
	}
}

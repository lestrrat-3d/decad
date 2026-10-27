// Package igesread parses the fixed-format records of an IGES file into entities
// (agent-docs/iges-import-design.md). It reads the sections of IGES 5.3, the global
// parameters that fix the delimiters and the unit, the directory entries and the parameter
// data of each entity, and interprets none of the geometry.
package igesread

import (
	"errors"
	"fmt"
	"strconv"
	"strings"
)

// ErrMalformed marks a file that is not well-formed IGES.
var ErrMalformed = errors.New("igesread: malformed IGES")

// Entity is one directory entry with its parameter data.
type Entity struct {
	// Seq is the sequence number of the entity's first directory line, the number the other entities point
	// at it with.
	Seq int
	// Type and Form are the entity type and form numbers.
	Type, Form int
	// Transform is the directory pointer to a transformation matrix, zero for none.
	Transform int
	// Params are the entity's parameters after the entity type number, as text with Hollerith strings and
	// the delimiters removed.
	Params []string
}

// Model is a parsed file.
type Model struct {
	pdelim, rdelim byte
	// Scale is the length of the file's unit in millimetres.
	Scale float64
	// ModelScale is the ratio of model space to physical space the global section states.
	ModelScale float64
	// Entities maps a directory sequence number to its entity.
	Entities map[int]*Entity
}

// units are the lengths in millimetres of the global section's unit flags 1 to 11.
var units = map[int]float64{
	1: 25.4, 2: 1, 4: 304.8, 5: 1609344, 6: 1000, 7: 1e6, 8: 0.0254, 9: 0.001, 10: 10, 11: 0.0000254,
}

// MaxLines bounds the records Parse reads.
const MaxLines = 4_000_000

// Parse reads an IGES file.
func Parse(data []byte) (*Model, error) {
	text := strings.ReplaceAll(string(data), "\r\n", "\n")
	text = strings.ReplaceAll(text, "\r", "\n")
	lines := strings.Split(strings.TrimRight(text, "\n"), "\n")
	if len(lines) > MaxLines {
		return nil, fmt.Errorf("%w: more than %d records", ErrMalformed, MaxLines)
	}
	var global strings.Builder
	var dir []string
	params := map[int][]string{}
	sawTerminate := false
	for n, line := range lines {
		if len(line) < 73 {
			line += strings.Repeat(" ", 80-len(line))
		}
		switch line[72] {
		case 'S':
		case 'G':
			global.WriteString(strings.TrimRight(line[:72], " "))
		case 'D':
			dir = append(dir, line)
		case 'P':
			de, err := strconv.Atoi(strings.TrimSpace(line[64:72]))
			if err != nil {
				return nil, fmt.Errorf("%w: record %d has no directory pointer", ErrMalformed, n+1)
			}
			params[de] = append(params[de], line[:64])
		case 'T':
			sawTerminate = true
		default:
			return nil, fmt.Errorf("%w: record %d has the section code %q", ErrMalformed, n+1, line[72])
		}
	}
	if !sawTerminate {
		return nil, fmt.Errorf("%w: the file has no terminate record", ErrMalformed)
	}
	m := &Model{Entities: map[int]*Entity{}}
	if err := m.global(global.String()); err != nil {
		return nil, err
	}
	if len(dir)%2 != 0 {
		return nil, fmt.Errorf("%w: the directory holds an odd number of records", ErrMalformed)
	}
	pd, rd := m.pdelim, m.rdelim
	for i := 0; i < len(dir); i += 2 {
		a, b := dir[i], dir[i+1]
		field := func(s string, k int) string { return strings.TrimSpace(s[8*k : 8*k+8]) }
		num := func(s string, k int) (int, error) {
			f := field(s, k)
			if f == "" {
				return 0, nil
			}
			return strconv.Atoi(f)
		}
		typ, err := num(a, 0)
		if err != nil {
			return nil, fmt.Errorf("%w: directory entry %d has no entity type", ErrMalformed, i/2+1)
		}
		if _, err := num(a, 1); err != nil {
			return nil, fmt.Errorf("%w: directory entry %d has no parameter pointer", ErrMalformed, i/2+1)
		}
		xform, err := num(a, 6)
		if err != nil {
			return nil, fmt.Errorf("%w: directory entry %d has a bad transformation pointer", ErrMalformed, i/2+1)
		}
		form, err := num(b, 4)
		if err != nil {
			return nil, fmt.Errorf("%w: directory entry %d has a bad form number", ErrMalformed, i/2+1)
		}
		seq := i + 1
		// The parameter records name their entity by the sequence number of its first directory line.
		lines := params[seq]
		if lines == nil {
			return nil, fmt.Errorf("%w: entity at directory line %d has no parameter records", ErrMalformed, seq)
		}
		fields, err := splitParams(strings.Join(lines, ""), pd, rd)
		if err != nil {
			return nil, fmt.Errorf("%w: entity at directory line %d: %s", ErrMalformed, seq, err)
		}
		if len(fields) == 0 {
			return nil, fmt.Errorf("%w: entity at directory line %d has no parameters", ErrMalformed, seq)
		}
		if t, err := strconv.Atoi(fields[0]); err != nil || t != typ {
			return nil, fmt.Errorf("%w: entity at directory line %d names type %d in its parameters and %d in the directory",
				ErrMalformed, seq, t, typ)
		}
		m.Entities[seq] = &Entity{Seq: seq, Type: typ, Form: form, Transform: xform, Params: fields[1:]}
	}
	return m, nil
}

// Reals reads parameters as numbers, accepting the Fortran D exponent.
func Reals(params []string) ([]float64, error) {
	out := make([]float64, len(params))
	for i, p := range params {
		v, err := Real(p)
		if err != nil {
			return nil, err
		}
		out[i] = v
	}
	return out, nil
}

// Real reads one parameter as a number, accepting the D exponent and an empty text as zero.
func Real(p string) (float64, error) {
	p = strings.TrimSpace(p)
	if p == "" {
		return 0, nil
	}
	p = strings.NewReplacer("D", "E", "d", "e").Replace(p)
	v, err := strconv.ParseFloat(p, 64)
	if err != nil {
		return 0, fmt.Errorf("%w: %q is not a number", ErrMalformed, p)
	}
	return v, nil
}

// Int reads one parameter as an integer.
func Int(p string) (int, error) {
	n, err := strconv.Atoi(strings.TrimSpace(p))
	if err != nil {
		return 0, fmt.Errorf("%w: %q is not an integer", ErrMalformed, p)
	}
	return n, nil
}

// splitParams splits a record's text at the parameter delimiter up to the record delimiter, taking a Hollerith
// string (nHtext) whole and trimming spaces from every other parameter. A record delimiter ends the list.
func splitParams(s string, pd, rd byte) ([]string, error) {
	var out []string
	i := 0
	for i <= len(s) {
		// One parameter starts at i.
		start := i
		for start < len(s) && s[start] == ' ' {
			start++
		}
		j := start
		digits := 0
		for j < len(s) && s[j] >= '0' && s[j] <= '9' {
			j++
			digits++
		}
		if digits > 0 && j < len(s) && s[j] == 'H' {
			n, _ := strconv.Atoi(s[start:j])
			if j+1+n > len(s) {
				return nil, fmt.Errorf("a Hollerith string runs past the record")
			}
			out = append(out, s[j+1:j+1+n])
			i = j + 1 + n
			for i < len(s) && s[i] == ' ' {
				i++
			}
		} else {
			end := start
			for end < len(s) && s[end] != pd && s[end] != rd {
				end++
			}
			out = append(out, strings.TrimSpace(s[start:end]))
			i = end
		}
		if i >= len(s) {
			return out, fmt.Errorf("the record has no record delimiter")
		}
		switch s[i] {
		case rd:
			return out, nil
		case pd:
			i++
		default:
			return nil, fmt.Errorf("unexpected %q after a parameter", s[i])
		}
	}
	return nil, fmt.Errorf("the record has no record delimiter")
}

// global reads the global section: the delimiters, the scale and the unit.
func (m *Model) global(s string) error {
	pd, rd := byte(','), byte(';')
	// The first two parameters are the delimiters: spelled out as Hollerith strings (1H,,1H;) or left empty for the
	// defaults (,,).
	if strings.HasPrefix(s, "1H") && len(s) >= 3 {
		pd = s[2]
		if len(s) >= 7 && (s[3] == pd || s[3] == ',') && s[4:6] == "1H" {
			rd = s[6]
		}
	}
	body := s
	m.pdelim, m.rdelim = pd, rd
	fields, err := globalFields(body, pd, rd)
	if err != nil {
		return fmt.Errorf("%w: global section: %s", ErrMalformed, err)
	}
	get := func(k int) string {
		if k-1 < len(fields) {
			return fields[k-1]
		}
		return ""
	}
	m.ModelScale = 1
	if v := get(13); v != "" {
		x, err := Real(v)
		if err != nil || !(x > 0) {
			return fmt.Errorf("%w: global section: the model space scale %q is not a positive number", ErrMalformed, v)
		}
		m.ModelScale = x
	}
	flag := 1
	if v := get(14); v != "" {
		f, err := Int(v)
		if err != nil {
			return fmt.Errorf("%w: global section: the unit flag %q is not an integer", ErrMalformed, v)
		}
		flag = f
	}
	scale, ok := units[flag]
	if !ok {
		return fmt.Errorf("%w: global section: the unit flag %d names no unit this reader knows", ErrMalformed, flag)
	}
	m.Scale = scale
	return nil
}

// globalFields splits the global section, whose first two parameters may be left out.
func globalFields(s string, pd, rd byte) ([]string, error) {
	// The first two parameters are the delimiters themselves; a file that leaves a delimiter out starts that
	// parameter empty. Parse them by hand and give the rest to splitParams.
	var out []string
	rest := s
	for k := range 2 {
		switch {
		case strings.HasPrefix(rest, "1H") && len(rest) >= 3:
			out = append(out, rest[2:3])
			rest = rest[3:]
		case len(rest) > 0 && rest[0] == pd:
			out = append(out, "")
		default:
			return nil, fmt.Errorf("the delimiter parameters are malformed")
		}
		// The first two parameters are set off by the default delimiter or by the one just defined.
		if len(rest) == 0 || (rest[0] != pd && rest[0] != rd && rest[0] != ',') {
			return nil, fmt.Errorf("a delimiter parameter is not followed by a delimiter")
		}
		if rest[0] == rd && k == 1 {
			return out, nil
		}
		rest = rest[1:]
	}
	more, err := splitParams(rest, pd, rd)
	if err != nil {
		return nil, err
	}
	return append(out, more...), nil
}

from cad2urdf.gui.app import parse_args


def test_parse_args_default_socket_is_none():
    ns = parse_args([])
    assert ns.control_socket is None


def test_parse_args_reads_control_socket():
    ns = parse_args(["--control-socket", "/tmp/x.sock"])
    assert ns.control_socket == "/tmp/x.sock"

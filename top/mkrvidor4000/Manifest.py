files = [
    "mkrvidor4000_top.sv",
    "arbiter.sv"
]

modules = {
    "git": [
        "https://github.com/hdl-util/hdmi::master",
        "https://github.com/hdl-util/sound::master",
        "https://github.com/hdl-util/vga-text-mode::master",
        "https://github.com/hdl-util/mipi-ccs::master",
        "https://github.com/hdl-util/mipi-csi-2::master",
        "https://github.com/hdl-util/sdram-controller::master",
        "https://github.com/hdl-util/clock-domain-crossing::master",
        "https://github.com/zappos23/jtag-vidor4000::main",
        "https://github.com/zappos23/uart::main",
        "https://github.com/zappos23/word_deserializer::main",
    ]
}

fetchto = "../../ip_cores"


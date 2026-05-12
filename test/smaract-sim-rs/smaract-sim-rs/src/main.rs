/*
 main program for the motor controller simulator
 parse the command line and hand over the execution
 into netcommand
*/

use crate::netcommand::Netcommand;
use clap::Parser;

mod netcommand;

#[derive(Parser)]
#[clap(author, version, about)]
pub struct Options {
    #[clap(short = 'd', long = "dump", help = "Dump communication")]
    dump: bool,
}

fn main() {
    let opts = Options::from_args();
    let netcommand = Netcommand::new(opts.dump);
    netcommand.simulate();
}

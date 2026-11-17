// A key registered as f64 cannot be read as anything else: this must not compile.
use bt::Blackboard;

fn main() {
    let mut bb = Blackboard::new();
    let time = bb.register::<f64>("sim_time");
    let _: Option<&usize> = bb.get(&time);
}

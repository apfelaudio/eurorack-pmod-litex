use std::collections::HashMap;
use std::sync::RwLock;
use lazy_static::lazy_static;

pub enum OptionValue {
    Integer(i32),
    Float(f32),
    Enum(String),
    // Add other types as needed
}

pub struct Option {
    name: String,
    current_value: OptionValue,
    min_value: OptionValue,
    max_value: OptionValue,
}

lazy_static! {
    static ref OPTIONS: RwLock<HashMap<String, Option>> = RwLock::new(HashMap::new());
}

pub fn set_option(name: &str, value: OptionValue) {
    let mut options = OPTIONS.write().unwrap();
    match options.get_mut(name) {
        Some(option) => option.current_value = value,
        None => println!("Option {} does not exist", name),
    }
}

pub fn get_option(name: &str) -> Option<OptionValue> {
    let options = OPTIONS.read().unwrap();
    match options.get(name) {
        Some(option) => Some(option.current_value.clone()),
        None => None,
    }
}

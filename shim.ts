// Define a namespace for your blocks (e.g., matching the 'name' in pxt.json)
namespace MathShims {

    /**
     * Calculates the sum of two numbers using optimized C++.
     * This function is exposed as a MakeCode block.
     * The key is the 'shim' annotation which tells the compiler to ignore the
     * TypeScript return (the 'return 0' below) and jump directly to the
     * C++ function named 'LowLatencyAdd'.
     */
    //% block="low latency $num1 plus $num2"
    //% num1.defl=10
    //% num2.defl=20
    //% shim=MathShims::LowLatencyAdd
    export function lowLatencyAdd(num1: number, num2: number): number {
        // This line is only a placeholder; the C++ function handles the real work.
        return 0;
    }

    /**
     * Calculates the difference of two numbers using optimized C++.
     */
    //% block="low latency $num1 minus $num2"
    //% num1.defl=30
    //% num2.defl=15
    //% shim=MathShims::LowLatencySubtract
    export function lowLatencySubtract(num1: number, num2: number): number {
        // Placeholder return
        return 0;
    }
}
using System;
using System.IO;

class GyroTest {
  static void Main(string[] args) {
    Console.WriteLine("Parsing stdin...");
    using (var s = Console.OpenStandardInput()) {
      using (var b = new BinaryReader(s)) {
        while (true) {
          double y = b.ReadDouble();
          double p = b.ReadDouble();
          double r = b.ReadDouble();
          Console.WriteLine("y={0:F3} p={1:F3} r={2:F3}", y, p, r);
        }
      }
    }
  }
}

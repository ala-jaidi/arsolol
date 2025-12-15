import 'package:flutter/material.dart';
import 'scanner.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    final scheme = const ColorScheme(
      brightness: Brightness.dark,
      primary: Color(0xFF0099FF),
      onPrimary: Colors.white,
      secondary: Color(0xFF40E0D0),
      onSecondary: Colors.black,
      tertiary: Color(0xFF80D8FF),
      onTertiary: Colors.black,
      error: Colors.redAccent,
      onError: Colors.white,
      surface: Color(0xFF0F172A),
      onSurface: Color(0xFFE2F3FF),
    );
    return MaterialApp(
      title: 'Scan Pied 3D',
      theme: ThemeData(
        useMaterial3: true,
        colorScheme: scheme,
        scaffoldBackgroundColor: scheme.surface,
        appBarTheme: const AppBarTheme(centerTitle: true, foregroundColor: Colors.white),
        snackBarTheme: SnackBarThemeData(behavior: SnackBarBehavior.floating, backgroundColor: scheme.surface, contentTextStyle: TextStyle(color: scheme.onSurface)),
        elevatedButtonTheme: ElevatedButtonThemeData(style: ElevatedButton.styleFrom(backgroundColor: scheme.primary, foregroundColor: scheme.onPrimary, shape: const StadiumBorder(), padding: const EdgeInsets.symmetric(vertical: 14))),
        dividerTheme: DividerThemeData(color: scheme.onSurface.withValues(alpha: 0.12)),
      ),
      home: const ScannerPage(),
    );
  }
}

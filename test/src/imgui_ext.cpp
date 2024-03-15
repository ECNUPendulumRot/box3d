
#include "gui/imgui_ext.hpp"

bool ImGui::ColorEdit3or4WithPalette(const char *label, float *color_in, bool with_alpha) {
  bool result = false;
  static bool saved_palette_initialized = false;
  static ImVec4 saved_palette[40];
  static ImVec4 backup_color;
  ImGui::PushID(label);
  int flags =
	ImGuiColorEditFlags_PickerHueWheel |
	ImGuiColorEditFlags_Float;

  if (!with_alpha) {
	flags |= ImGuiColorEditFlags_NoAlpha;
  }

  ImVec4 &color = *(ImVec4 *)color_in;

  if (!saved_palette_initialized) {

	for (int n = 0; n < 8; n++) {
	  saved_palette[n].x = 0.0f;
	  saved_palette[n].y = 0.0f;
	  saved_palette[n].z = 0.0f;
	}

	saved_palette[0] = ImVec4(0.0f, 0.0f, 0.0f, 1.0f);
	saved_palette[1] = ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
	saved_palette[2] = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
	saved_palette[3] = ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
	saved_palette[4] = ImVec4(1.0f, 1.0f, 0.0f, 1.0f);
	saved_palette[5] = ImVec4(0.0f, 0.0f, 1.0f, 1.0f);
	saved_palette[6] = ImVec4(0.0f, 1.0f, 0.0f, 1.0f);
	saved_palette[7] = ImVec4(0.0f, 1.0f, 1.0f, 1.0f);

	for (int n = 0; n < 32; n++) {
	  ImGui::ColorConvertHSVtoRGB(
		float(n) / 31.0f, 0.8f, 0.8f,
		saved_palette[n + 8].x,
		saved_palette[n + 8].y,
		saved_palette[n + 8].z
	  );
	}
	saved_palette_initialized = true;
  }

  bool open_popup = ImGui::ColorButton(label, color, flags);

  if (label[0] != '#') {
	ImGui::SameLine();
	ImGui::Text("%s", label);
  }
  if (open_popup) {
	ImGui::OpenPopup("##PickerPopup");
	backup_color = color;
  }
  if (ImGui::BeginPopup("##PickerPopup")) {
	if (label[0] != '#') {
	  ImGui::Text("%s", label);
	  ImGui::Separator();
	}
	if (ImGui::ColorPicker4(
	  "##picker", (float *)&color,
	  flags | ImGuiColorEditFlags_NoSidePreview
	  | ImGuiColorEditFlags_NoSmallPreview
	  )
	  ) {
	  result = true;
	}
	ImGui::SameLine();
	ImGui::BeginGroup();
	ImGui::Text("Current");
	ImGui::ColorButton(
	  "##current", color,
	  ImGuiColorEditFlags_NoPicker |
	  ImGuiColorEditFlags_AlphaPreviewHalf,
	  ImVec2(60, 40)
	);
	ImGui::Text("Previous");
	if (ImGui::ColorButton(
	  "##previous", backup_color,
	  ImGuiColorEditFlags_NoPicker |
	  ImGuiColorEditFlags_AlphaPreviewHalf,
	  ImVec2(60, 40))
	  ) {
	  color = backup_color;
	  result = true;
	}
	ImGui::Separator();
	ImGui::Text("Palette");

	int nb_btn_per_row = 8;
	float btn_size = 20.0;

	for (int n = 0; n < 40; n++) {
	  ImGui::PushID(n);
	  if ((n % nb_btn_per_row) != 0) {
		ImGui::SameLine(0.0f, ImGui::GetStyle().ItemSpacing.y);
	  }
	  if (ImGui::ColorButton(
		"##palette",
		saved_palette[n],
		ImGuiColorEditFlags_NoPicker |
		ImGuiColorEditFlags_NoTooltip,
		ImVec2(btn_size, btn_size))
		) {
		color = ImVec4(
		  saved_palette[n].x,
		  saved_palette[n].y,
		  saved_palette[n].z,
		  color.w
		); // Preserve alpha!
		result = true;
	  }
	  ImGui::PopID();
	}
	ImGui::Separator();
	if (ImGui::Button(
	  "OK", ImVec2(-1, -1)
	  )
	  ) {
	  ImGui::CloseCurrentPopup();
	}
	ImGui::EndGroup();
	ImGui::EndPopup();
  }
  ImGui::PopID();
  return result;
}


bool ImGui::ColorEdit3WithPalette(const char *label, float *color_in) {
  return ColorEdit3or4WithPalette(label, color_in, false);
}

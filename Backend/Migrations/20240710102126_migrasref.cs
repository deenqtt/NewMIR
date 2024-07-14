using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace Backend.Migrations
{
    /// <inheritdoc />
    public partial class migrasref : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropColumn(
                name: "OriginX",
                table: "Maps");

            migrationBuilder.DropColumn(
                name: "OriginY",
                table: "Maps");

            migrationBuilder.DropColumn(
                name: "OriginZ",
                table: "Maps");

            migrationBuilder.AddColumn<string>(
                name: "Origin",
                table: "Maps",
                type: "TEXT",
                nullable: false,
                defaultValue: "");
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropColumn(
                name: "Origin",
                table: "Maps");

            migrationBuilder.AddColumn<double>(
                name: "OriginX",
                table: "Maps",
                type: "REAL",
                nullable: false,
                defaultValue: 0.0);

            migrationBuilder.AddColumn<double>(
                name: "OriginY",
                table: "Maps",
                type: "REAL",
                nullable: false,
                defaultValue: 0.0);

            migrationBuilder.AddColumn<double>(
                name: "OriginZ",
                table: "Maps",
                type: "REAL",
                nullable: false,
                defaultValue: 0.0);
        }
    }
}
